#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
USB_SCRIPT="$SCRIPT_DIR/w9ksb_usb.sh"
CONTROLLER_SCRIPT="$SCRIPT_DIR/w9ksb_controller.py"
USB_SERVICE_PATH="/etc/systemd/system/w9ksb-usb.service"
CONTROLLER_SERVICE_PATH="/etc/systemd/system/w9ksb_controller.service"

if [[ $EUID -ne 0 ]]; then
    echo "Run this installer with sudo."
    exit 1
fi

if [[ ! -f "$USB_SCRIPT" ]]; then
    echo "Missing USB setup script: $USB_SCRIPT"
    exit 1
fi

if [[ ! -f "$CONTROLLER_SCRIPT" ]]; then
    echo "Missing controller script: $CONTROLLER_SCRIPT"
    exit 1
fi

resolve_target_user() {
    if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
        echo "$SUDO_USER"
        return
    fi

    local active_user
    active_user="$(logname 2>/dev/null || true)"
    if [[ -n "$active_user" && "$active_user" != "root" ]]; then
        echo "$active_user"
        return
    fi

    echo "Could not determine the non-root user who owns this repo."
    echo "Run the installer with: sudo ./install.sh"
    exit 1
}

TARGET_USER="$(resolve_target_user)"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"

if [[ -z "$TARGET_HOME" || ! -d "$TARGET_HOME" ]]; then
    echo "Could not determine home directory for user: $TARGET_USER"
    exit 1
fi

ensure_boot_files() {
    if [[ -f /boot/firmware/config.txt ]]; then
        BOOT_CONFIG="/boot/firmware/config.txt"
    elif [[ -f /boot/config.txt ]]; then
        BOOT_CONFIG="/boot/config.txt"
    else
        echo "Could not find config.txt"
        exit 1
    fi

    if [[ -f /boot/firmware/cmdline.txt ]]; then
        BOOT_CMDLINE="/boot/firmware/cmdline.txt"
    elif [[ -f /boot/cmdline.txt ]]; then
        BOOT_CMDLINE="/boot/cmdline.txt"
    else
        echo "Could not find cmdline.txt"
        exit 1
    fi
}

backup_file_once() {
    local file="$1"
    local backup="$file.w9ksb.bak"
    if [[ ! -f "$backup" ]]; then
        cp -a "$file" "$backup"
    fi
}

ensure_dwc2_overlay() {
    if ! grep -q '^dtoverlay=dwc2$' "$BOOT_CONFIG"; then
        printf '\n# W9KSB USB gadget\ndtoverlay=dwc2\n' >> "$BOOT_CONFIG"
        echo "Added dtoverlay=dwc2 to $BOOT_CONFIG"
    else
        echo "dtoverlay=dwc2 already present"
    fi
}

ensure_cmdline_module() {
    if grep -q 'modules-load=dwc2' "$BOOT_CMDLINE"; then
        echo "modules-load=dwc2 already present"
        return
    fi

    if grep -q '\<rootwait\>' "$BOOT_CMDLINE"; then
        sed -i 's/\<rootwait\>/rootwait modules-load=dwc2/' "$BOOT_CMDLINE"
    else
        echo " modules-load=dwc2" >> "$BOOT_CMDLINE"
        tr -s ' ' < "$BOOT_CMDLINE" > "$BOOT_CMDLINE.tmp"
        mv "$BOOT_CMDLINE.tmp" "$BOOT_CMDLINE"
    fi
    echo "Added modules-load=dwc2 to $BOOT_CMDLINE"
}

install_packages() {
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        python3 \
        python3-pip \
        python3-serial \
        python3-flask \
        python3-rpi.gpio \
        python3-smbus \
        i2c-tools \
        psmisc \
        lsof

    python3 -m pip install --break-system-packages -U pyserial flask RPLCD
}

enable_i2c() {
    if command -v raspi-config >/dev/null 2>&1; then
        raspi-config nonint do_i2c 0 || true
    fi
}

create_services() {
    cat > "$USB_SERVICE_PATH" <<SERVICE
[Unit]
Description=W9KSB USB Gadget Setup
DefaultDependencies=no
After=local-fs.target sys-kernel-config.mount
Before=sysinit.target
ConditionPathExists=$USB_SCRIPT

[Service]
Type=oneshot
ExecStart=/bin/bash $USB_SCRIPT
RemainAfterExit=yes

[Install]
WantedBy=sysinit.target
SERVICE

    cat > "$CONTROLLER_SERVICE_PATH" <<SERVICE
[Unit]
Description=W9KSB Controller
After=network-online.target w9ksb-usb.service
Wants=network-online.target
Requires=w9ksb-usb.service
ConditionPathExists=$CONTROLLER_SCRIPT

[Service]
Type=simple
WorkingDirectory=$SCRIPT_DIR
ExecStart=/usr/bin/python3 $CONTROLLER_SCRIPT
Restart=always
RestartSec=2
User=root
Environment=HOME=$TARGET_HOME
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
SERVICE
}

main() {
    echo "Repo directory: $SCRIPT_DIR"
    echo "Repo owner user: $TARGET_USER"
    echo "Repo owner home: $TARGET_HOME"

    chmod +x "$USB_SCRIPT"
    chmod +x "$CONTROLLER_SCRIPT"

    ensure_boot_files
    backup_file_once "$BOOT_CONFIG"
    backup_file_once "$BOOT_CMDLINE"
    ensure_dwc2_overlay
    ensure_cmdline_module

    install_packages
    enable_i2c

    mkdir -p "$TARGET_HOME/.w9ksb"
    chown -R "$TARGET_USER:$TARGET_USER" "$TARGET_HOME/.w9ksb"

    create_services

    systemctl daemon-reload
    systemctl enable w9ksb-usb.service
    systemctl enable w9ksb_controller.service

    echo
    echo "Install complete."
    echo ""
    echo "Files used:"
    echo "  USB script:        $USB_SCRIPT"
    echo "  Controller script: $CONTROLLER_SCRIPT"
    echo ""
    echo "Services installed:"
    echo "  $USB_SERVICE_PATH"
    echo "  $CONTROLLER_SERVICE_PATH"
    echo ""
    echo "Next step: reboot the Pi."
    echo "After reboot, check:"
    echo "  systemctl status w9ksb-usb.service"
    echo "  systemctl status w9ksb_controller.service"
}

main "$@"
