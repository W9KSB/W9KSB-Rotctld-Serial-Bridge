#!/usr/bin/env python3
import argparse
import json
import math
import queue
import socket
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional, Tuple

from flask import Flask, Response, redirect, request

try:
    import serial
except Exception:
    serial = None

try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None

try:
    from RPLCD.i2c import CharLCD
except Exception:
    CharLCD = None

APP_DIR = Path.home() / ".w9ksb"
CONFIG_PATH = APP_DIR / "config.json"

LOG_LINES = 40
LOG_LINE_MAX = 220

PRIMARY_SERIAL_DEFAULT = "/dev/ttyGS0"
SECONDARY_SERIAL_DEFAULT = "/dev/ttyGS1"
HID_DEFAULT = "/dev/hidg0"

LCD_ADDR_DEFAULT = 0x27
LCD_COLS = 20
LCD_ROWS = 4

ENC_A_DEFAULT = 17
ENC_B_DEFAULT = 27
ENC_SW_DEFAULT = 22

SERIAL_READ_TIMEOUT = 0.10
SERIAL_REOPEN_DELAY = 1.0
ROT_CONNECT_TIMEOUT = 1.2
ROT_RW_TIMEOUT = 0.7
RIG_CONNECT_TIMEOUT = 1.2
RIG_RW_TIMEOUT = 0.7

BTN_DEBOUNCE_SEC = 0.15
ENC_POLL_SEC = 0.0015
ENC_STEP_MIN_SEC = 0.015
DEFAULT_BTN_KEY = "ENTER"

DEBUG = False
app = Flask(__name__)


def dprint(*args):
    if DEBUG:
        print("[DEBUG]", *args, flush=True)


@dataclass
class Config:
    rot_enabled: bool = True
    rot_host: str = "192.168.1.50"
    rot_port: int = 4533
    safety_enabled: bool = True

    sdr_enabled: bool = True
    sdr_host: str = "192.168.1.123"
    sdr_port: int = 4532
    sdr_poll_ms: int = 750
    mode_poll_ms: int = 1500
    ptt_poll_ms: int = 2000
    rigctl_enabled: bool = True

    primary_serial: str = PRIMARY_SERIAL_DEFAULT
    primary_baud: int = 57600
    secondary_serial: str = SECONDARY_SERIAL_DEFAULT
    secondary_baud: int = 57600

    hid_path: str = HID_DEFAULT
    lcd_enabled: bool = True
    lcd_addr: int = LCD_ADDR_DEFAULT

    encoder_enabled: bool = True
    enc_a_pin: int = ENC_A_DEFAULT
    enc_b_pin: int = ENC_B_DEFAULT
    enc_sw_pin: int = ENC_SW_DEFAULT
    btn_key: str = DEFAULT_BTN_KEY


class RingLog:
    def __init__(self, max_lines: int = LOG_LINES):
        self.lines = deque(maxlen=max_lines)
        self.lock = threading.Lock()

    def add(self, text: str) -> None:
        s = str(text)
        if len(s) > LOG_LINE_MAX:
            s = s[:LOG_LINE_MAX]
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {s}"
        with self.lock:
            self.lines.append(line)
        if DEBUG:
            print(line, flush=True)

    def get(self, n: int = LOG_LINES) -> str:
        with self.lock:
            data = list(self.lines)[-n:]
        return "\n".join(data) + ("\n" if data else "")


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.hostname = socket.gethostname()
        self.ssid = "--"
        self.ip = "--"
        self.wifi_connected = False

        self.last_pc_az = math.nan
        self.last_pc_el = math.nan
        self.last_rot_az = math.nan
        self.last_rot_el = math.nan

        self.primary_serial_open = False
        self.primary_serial_error = ""
        self.secondary_serial_open = False
        self.secondary_serial_error = ""

        self.rot_connected = False
        self.rig_connected = False

        self.enc_freq_ctrl_mode = False
        self.hid_available = False
        self.hid_queue_depth = 0
        self.hid_sent = 0
        self.hid_dropped = 0
        self.hid_last_key = "-"
        self.hid_status = "Not started"

        self.encoder_status = "Not started"
        self.encoder_raw_a = "?"
        self.encoder_raw_b = "?"
        self.encoder_raw_sw = "?"
        self.encoder_last_dir = "-"
        self.encoder_position = 0
        self.encoder_exceptions = 0

        self.vfo_a_hz = 145800000
        self.vfo_b_hz = 145800000
        self.rx_vfo = 0
        self.tx_vfo = 0
        self.split_enabled = False
        self.mode_code = 4
        self.ptt_state = False
        self.ai_mode = 0

        self.last_cat_cmd = ""
        self.last_rig_cmd = ""


cfg = Config()
state = State()

logPC = RingLog()
logROT = RingLog()
logCAT = RingLog()
logSDR = RingLog()
logENC = RingLog()
logSYS = RingLog()

primary_serial = None
secondary_serial = None
primary_lock = threading.Lock()
secondary_lock = threading.Lock()

rot_sock: Optional[socket.socket] = None
rig_sock: Optional[socket.socket] = None
rot_lock = threading.Lock()
rig_lock = threading.Lock()

hid_queue: queue.Queue[str] = queue.Queue(maxsize=128)

stop_event = threading.Event()

lcd = None
lcd_cache = [""] * LCD_ROWS
lcd_lock = threading.Lock()


def ensure_app_dir() -> None:
    APP_DIR.mkdir(parents=True, exist_ok=True)


def save_config() -> None:
    ensure_app_dir()
    CONFIG_PATH.write_text(json.dumps(asdict(cfg), indent=2))


def load_config() -> None:
    global cfg
    ensure_app_dir()
    if CONFIG_PATH.exists():
        merged = asdict(Config())
        merged.update(json.loads(CONFIG_PATH.read_text()))
        cfg = Config(**merged)
    else:
        save_config()


def html_escape(s: str) -> str:
    return (str(s).replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;")
            .replace("'", "&#39;"))


def fmt3(v: float) -> str:
    if v is None or not math.isfinite(v):
        return "---"
    return f"{max(0, min(999, int(round(v)))):03d}"


def fmt11(v: int) -> str:
    return f"{int(v):011d}"




def cached_rot_position() -> Tuple[bool, float, float]:
    with state.lock:
        az = state.last_rot_az
        el = state.last_rot_el
    return math.isfinite(az) and math.isfinite(el), az, el


def current_rot_position(prefer_cached: bool = True) -> Tuple[bool, float, float]:
    if prefer_cached:
        ok, az, el = cached_rot_position()
        if ok:
            return True, az, el
    ok, az, el = rot_get_position()
    if ok:
        return True, az, el
    ok, az, el = cached_rot_position()
    if ok:
        return True, az, el
    return False, 0.0, 0.0


def gs232_position_reply(az: float, el: float) -> str:
    return f"AZ={fmt3(az)} EL={fmt3(el)}"

def get_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "--"


def get_ssid() -> str:
    try:
        out = subprocess.check_output(["iwgetid", "-r"], stderr=subprocess.DEVNULL, text=True).strip()
        return out or "--"
    except Exception:
        return "--"


def network_worker():
    while not stop_event.is_set():
        ip = get_ip()
        ssid = get_ssid()
        with state.lock:
            state.ip = ip
            state.ssid = ssid
            state.wifi_connected = ip != "--"
        time.sleep(5)


def wifi_status_line() -> str:
    with state.lock:
        return "Wifi: Connected" if state.wifi_connected else "Wifi: Disconnected"


def ip_status_line() -> str:
    with state.lock:
        return f"IP: {state.ip}"


def network_status_block() -> str:
    with state.lock:
        return (
            f"<div class='col'><div class='mono'><b>Hostname</b><br>{html_escape(state.hostname)}</div></div>"
            f"<div class='col'><div class='mono'><b>Network Name</b><br>{html_escape(state.ssid)}</div></div>"
            f"<div class='col'><div class='mono'><b>IP Address</b><br>{html_escape(state.ip)}</div></div>"
        )


def mode_code_to_hamlib(code: int) -> str:
    return {1: "LSB", 2: "USB", 3: "CW", 4: "FM", 5: "AM", 6: "RTTY", 7: "CWR", 9: "RTTYR"}.get(code, "FM")


def hamlib_mode_to_code(mode: str) -> int:
    return {"LSB": 1, "USB": 2, "CW": 3, "FM": 4, "AM": 5, "RTTY": 6, "CWR": 7, "RTTYR": 9}.get(mode.strip().upper(), 4)


def open_usb_serial(path: str, baud: int):
    if serial is None:
        raise RuntimeError("pyserial not installed")

    ser = serial.Serial(
        port=path,
        baudrate=baud,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=SERIAL_READ_TIMEOUT,
        write_timeout=1.0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        exclusive=False,
    )

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    try:
        ser.reset_output_buffer()
    except Exception:
        pass

    return ser


def rot_disconnect() -> None:
    global rot_sock
    with rot_lock:
        if rot_sock is not None:
            try:
                rot_sock.close()
            except Exception:
                pass
            rot_sock = None
    with state.lock:
        state.rot_connected = False
    logROT.add("ROT disconnected")


def rot_ensure_connected() -> bool:
    global rot_sock
    if not cfg.rot_enabled:
        return False
    with rot_lock:
        if rot_sock is not None:
            return True
        try:
            s = socket.create_connection((cfg.rot_host, int(cfg.rot_port)), timeout=ROT_CONNECT_TIMEOUT)
            s.settimeout(ROT_RW_TIMEOUT)
            rot_sock = s
            with state.lock:
                state.rot_connected = True
            logROT.add(f"ROT Connected {cfg.rot_host}:{cfg.rot_port}")
            return True
        except Exception as e:
            rot_sock = None
            with state.lock:
                state.rot_connected = False
            logROT.add(f"ROT connect failed: {e}")
            return False


def rot_send_line(line: str) -> bool:
    global rot_sock
    if not rot_ensure_connected():
        return False
    with rot_lock:
        try:
            assert rot_sock is not None
            rot_sock.sendall((line + "\n").encode())
            logROT.add(f">> {line}")
            return True
        except Exception as e:
            logROT.add(f"ROT send failed: {e}")
            try:
                rot_sock.close()
            except Exception:
                pass
            rot_sock = None
            with state.lock:
                state.rot_connected = False
            return False


def rot_read_line(timeout: float = ROT_RW_TIMEOUT) -> Optional[str]:
    global rot_sock
    if not rot_ensure_connected():
        return None
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        try:
            with rot_lock:
                assert rot_sock is not None
                chunk = rot_sock.recv(1)
            if not chunk:
                return None
            if chunk == b"\r":
                continue
            if chunk == b"\n":
                if buf:
                    return buf.decode(errors="ignore")
                continue
            if len(buf) < 200:
                buf += chunk
        except socket.timeout:
            continue
        except Exception as e:
            dprint("rot_read_line exception", e)
            return None
    return None


def rot_get_position() -> Tuple[bool, float, float]:
    if not rot_send_line("p"):
        return False, math.nan, math.nan
    l1 = rot_read_line()
    l2 = rot_read_line()
    if l1 is None or l2 is None:
        logROT.add("<< timeout")
        return False, math.nan, math.nan
    try:
        # Project-specific mapping: this rotctld path reports elevation first
        # and azimuth second, so translate back to GS-232 AZ/EL here.
        az = float(l1)
        el = float(l2)
    except Exception:
        return False, math.nan, math.nan
    with state.lock:
        state.last_rot_az = az
        state.last_rot_el = el
    logROT.add(f"<< {l1}")
    logROT.add(f"<< {l2}")
    dprint(f"[ROT] translated position -> AZ={az:.3f} EL={el:.3f}")
    return True, az, el


def rot_set_position(az: float, el: float) -> bool:
    if cfg.safety_enabled:
        az = max(0.0, min(360.0, az))
        el = max(0.0, min(90.0, el))
    # Outbound rotctld set commands are azimuth first, elevation second.
    ok = rot_send_line(f"P {az:.1f} {el:.1f}")
    if ok:
        with state.lock:
            state.last_rot_az = az
            state.last_rot_el = el
        dprint(f"[ROT] translated setpoint AZ={az:.3f} EL={el:.3f} -> P {az:.1f} {el:.1f}")
    return ok


def rig_disconnect() -> None:
    global rig_sock
    with rig_lock:
        if rig_sock is not None:
            try:
                rig_sock.close()
            except Exception:
                pass
            rig_sock = None
    with state.lock:
        state.rig_connected = False
    logSDR.add("SDR disconnected")


def rig_ensure_connected() -> bool:
    global rig_sock
    if not cfg.sdr_enabled or not cfg.rigctl_enabled:
        return False
    with rig_lock:
        if rig_sock is not None:
            return True
        try:
            s = socket.create_connection((cfg.sdr_host, int(cfg.sdr_port)), timeout=RIG_CONNECT_TIMEOUT)
            s.settimeout(RIG_RW_TIMEOUT)
            rig_sock = s
            with state.lock:
                state.rig_connected = True
            logSDR.add(f"SDR Connected {cfg.sdr_host}:{cfg.sdr_port}")
            return True
        except Exception as e:
            rig_sock = None
            with state.lock:
                state.rig_connected = False
            logSDR.add(f"SDR connect failed: {e}")
            return False


def rig_flush_input() -> None:
    global rig_sock
    if not rig_ensure_connected():
        return
    with rig_lock:
        try:
            rig_sock.setblocking(False)
            while True:
                data = rig_sock.recv(1024)
                if not data:
                    break
        except Exception:
            pass
        finally:
            if rig_sock is not None:
                rig_sock.settimeout(RIG_RW_TIMEOUT)


def rig_command(cmd: str, expect_two_lines: bool = False) -> Tuple[bool, str, str]:
    global rig_sock
    if not rig_ensure_connected():
        return False, "", ""
    rig_flush_input()
    with state.lock:
        state.last_rig_cmd = cmd
    logSDR.add(f">> {cmd}")
    with rig_lock:
        try:
            assert rig_sock is not None
            rig_sock.sendall((cmd + "\n").encode())
        except Exception as e:
            logSDR.add(f"SDR send failed: {e}")
            try:
                rig_sock.close()
            except Exception:
                pass
            rig_sock = None
            with state.lock:
                state.rig_connected = False
            return False, "", ""

    deadline = time.time() + RIG_RW_TIMEOUT
    lines = []
    buf = b""
    while time.time() < deadline:
        try:
            with rig_lock:
                assert rig_sock is not None
                b = rig_sock.recv(1)
            if not b:
                break
            if b == b"\r":
                continue
            if b == b"\n":
                if buf:
                    line = buf.decode(errors="ignore").strip()
                    if line:
                        lines.append(line)
                        logSDR.add(f"<< {line}")
                    buf = b""
                    if not expect_two_lines and len(lines) >= 1:
                        return True, lines[0], ""
                    if expect_two_lines and len(lines) >= 2:
                        return True, lines[0], lines[1]
                continue
            if len(buf) < 200:
                buf += b
        except socket.timeout:
            continue
        except Exception as e:
            dprint("rig_command exception", e)
            return False, "", ""
    if len(lines) >= 1:
        return True, lines[0], lines[1] if len(lines) > 1 else ""
    return False, "", ""


def rig_set_freq(hz: int) -> bool:
    ok, l1, _ = rig_command(f"F {int(hz)}")
    return ok and l1 == "RPRT 0"


def rig_get_freq() -> Optional[int]:
    ok, l1, _ = rig_command("f")
    if ok and l1.isdigit():
        return int(l1)
    return None


def rig_set_mode(code: int) -> bool:
    ok, l1, _ = rig_command(f"M {mode_code_to_hamlib(code)} 0")
    return ok and l1 == "RPRT 0"


def rig_get_mode() -> Optional[int]:
    ok, l1, _ = rig_command("m", expect_two_lines=True)
    if ok and l1:
        return hamlib_mode_to_code(l1)
    return None


def rig_set_ptt(on: bool) -> bool:
    ok, l1, _ = rig_command(f"T {1 if on else 0}")
    return ok and l1 == "RPRT 0"


def rig_get_ptt() -> Optional[bool]:
    ok, l1, _ = rig_command("t")
    if ok and l1:
        return l1 in ("1", "2", "3")
    return None


def hid_key_to_report(key_name: str):
    mapping = {
        "ENTER": (0x00, 0x28), "RETURN": (0x00, 0x28), "SPACE": (0x00, 0x2C), "TAB": (0x00, 0x2B),
        "BACKSPACE": (0x00, 0x2A), "BKSP": (0x00, 0x2A), "ESC": (0x00, 0x29), "ESCAPE": (0x00, 0x29),
        "UP": (0x00, 0x52), "DOWN": (0x00, 0x51), "+": (0x02, 0x2E), "-": (0x00, 0x2D),
    }
    k = key_name.strip().upper()
    if len(k) == 1 and "A" <= k <= "Z":
        mapping[k] = (0x00, 0x04 + (ord(k) - ord("A")))
    return mapping.get(k)


def hid_send_immediate(key_name: str) -> bool:
    item = hid_key_to_report(key_name)
    if item is None:
        logENC.add(f"HID unknown key: {key_name}")
        return False
    mod, code = item
    try:
        with open(cfg.hid_path, "wb") as f:
            f.write(bytes([mod, 0, code, 0, 0, 0, 0, 0]))
            f.write(bytes(8))
        with state.lock:
            state.hid_available = True
            state.hid_sent += 1
            state.hid_last_key = key_name
        return True
    except Exception as e:
        with state.lock:
            state.hid_available = False
            state.hid_status = f"Write failed: {e}"
        logENC.add(f"HID write failed: {e}")
        return False


def enqueue_hid(key_name: str):
    try:
        hid_queue.put_nowait(key_name)
        with state.lock:
            state.hid_queue_depth = hid_queue.qsize()
            state.hid_status = f"Queued {key_name}"
        dprint("hid queued", key_name, "depth", hid_queue.qsize())
        return True
    except queue.Full:
        with state.lock:
            state.hid_dropped += 1
            state.hid_status = "Queue full"
        logENC.add(f"HID queue full, dropped {key_name}")
        return False


def hid_worker():
    with state.lock:
        state.hid_status = "Worker started"
    logSYS.add("HID worker started")
    while not stop_event.is_set():
        try:
            key_name = hid_queue.get(timeout=0.2)
        except queue.Empty:
            with state.lock:
                state.hid_queue_depth = 0
            continue

        with state.lock:
            state.hid_queue_depth = hid_queue.qsize()
            state.hid_status = f"Sending {key_name}"

        ok = hid_send_immediate(key_name)
        if ok:
            logENC.add(f"HID sent {key_name}")
            with state.lock:
                state.hid_status = f"Sent {key_name}"
        else:
            with state.lock:
                state.hid_status = f"Failed {key_name}"

        hid_queue.task_done()


def send_key_plus():
    if enqueue_hid("+"):
        logENC.add("TX Freq + queued")


def send_key_minus():
    if enqueue_hid("-"):
        logENC.add("TX Freq - queued")


def send_key_lane_up():
    if enqueue_hid("UP"):
        logENC.add("TX Lane UP queued")


def send_key_lane_down():
    if enqueue_hid("DOWN"):
        logENC.add("TX Lane DOWN queued")


def send_button_key():
    if enqueue_hid(cfg.btn_key):
        logENC.add(f"TX Button {cfg.btn_key} queued")


def toggle_encoder_mode():
    with state.lock:
        state.enc_freq_ctrl_mode = not state.enc_freq_ctrl_mode
        mode = state.enc_freq_ctrl_mode
    logENC.add("ENC Mode -> Frequency Ctrl" if mode else "ENC Mode -> CALIBRATE")


def encoder_worker():
    if not cfg.encoder_enabled:
        with state.lock:
            state.encoder_status = "Disabled in config"
        logENC.add("Encoder disabled in config")
        return
    if GPIO is None:
        with state.lock:
            state.encoder_status = "RPi.GPIO not installed"
        logENC.add("RPi.GPIO not installed")
        return

    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(cfg.enc_a_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(cfg.enc_b_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(cfg.enc_sw_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    except Exception as e:
        with state.lock:
            state.encoder_status = f"GPIO init failed: {e}"
        logENC.add(f"GPIO init failed: {e}")
        return

    try:
        a = GPIO.input(cfg.enc_a_pin)
        b = GPIO.input(cfg.enc_b_pin)
        sw = GPIO.input(cfg.enc_sw_pin)

        prev_a = a
        prev_sw = sw
        last_emit = 0.0
        last_btn = 0.0

        with state.lock:
            state.encoder_status = f"Active on GPIO {cfg.enc_a_pin}/{cfg.enc_b_pin}/{cfg.enc_sw_pin}"
            state.encoder_raw_a = str(a)
            state.encoder_raw_b = str(b)
            state.encoder_raw_sw = str(sw)

        logENC.add(f"Encoder active on GPIO {cfg.enc_a_pin}/{cfg.enc_b_pin}/{cfg.enc_sw_pin}")
        if DEBUG:
            dprint(f"encoder init a={a} b={b} sw={sw}")

        while not stop_event.is_set():
            a = GPIO.input(cfg.enc_a_pin)
            b = GPIO.input(cfg.enc_b_pin)
            sw = GPIO.input(cfg.enc_sw_pin)
            now = time.time()

            with state.lock:
                state.encoder_raw_a = str(a)
                state.encoder_raw_b = str(b)
                state.encoder_raw_sw = str(sw)

            if sw == GPIO.LOW and prev_sw == GPIO.HIGH and (now - last_btn) >= BTN_DEBOUNCE_SEC:
                toggle_encoder_mode()
                send_button_key()
                logENC.add("ENC Button")
                if DEBUG:
                    dprint("enc button press")
                last_btn = now
            prev_sw = sw

            if a == GPIO.HIGH and prev_a == GPIO.LOW and (now - last_emit) >= ENC_STEP_MIN_SEC:
                clockwise = (b == GPIO.HIGH)
                with state.lock:
                    freq_mode = state.enc_freq_ctrl_mode
                    state.encoder_last_dir = "CW" if clockwise else "CCW"
                    state.encoder_position += 1 if clockwise else -1

                if clockwise:
                    if freq_mode:
                        send_key_lane_up()
                    else:
                        send_key_plus()
                    logENC.add("ENC Dial CW")
                    if DEBUG:
                        dprint(f"enc dial cw a={a} b={b} mode={'freq' if freq_mode else 'cal'}")
                else:
                    if freq_mode:
                        send_key_lane_down()
                    else:
                        send_key_minus()
                    logENC.add("ENC Dial CCW")
                    if DEBUG:
                        dprint(f"enc dial ccw a={a} b={b} mode={'freq' if freq_mode else 'cal'}")

                last_emit = now

            prev_a = a
            time.sleep(ENC_POLL_SEC)

    except Exception as e:
        with state.lock:
            state.encoder_status = f"Encoder loop failed: {e}"
            state.encoder_exceptions += 1
        logENC.add(f"Encoder loop failed: {e}")
        logSYS.add(f"Encoder loop failed: {e}")
        if DEBUG:
            raise


def init_lcd():
    global lcd
    if not cfg.lcd_enabled:
        logSYS.add("LCD disabled")
        return
    if CharLCD is None:
        logSYS.add("RPLCD not installed")
        return
    try:
        lcd = CharLCD('PCF8574', cfg.lcd_addr, cols=LCD_COLS, rows=LCD_ROWS)
        lcd.clear()
        logSYS.add(f"LCD initialized at 0x{cfg.lcd_addr:02X}")
    except Exception as e:
        lcd = None
        logSYS.add(f"LCD init failed: {e}")


def lcd_write_line(row: int, text: str):
    global lcd_cache
    if lcd is None:
        return
    out = str(text)[:LCD_COLS].ljust(LCD_COLS)
    if lcd_cache[row] == out:
        return
    lcd_cache[row] = out
    try:
        with lcd_lock:
            lcd.cursor_pos = (row, 0)
            lcd.write_string(out)
    except Exception as e:
        logSYS.add(f"LCD write failed: {e}")


def lcd_center_20(s: str) -> str:
    s = str(s)
    if len(s) >= 20:
        return s[:20]
    left = (20 - len(s)) // 2
    return ((" " * left) + s).ljust(20)


def lcd_worker():
    while not stop_event.is_set():
        with state.lock:
            freq_mode = state.enc_freq_ctrl_mode
            az = state.last_rot_az
            el = state.last_rot_el
        lcd_write_line(0, lcd_center_20(wifi_status_line()))
        lcd_write_line(1, ip_status_line()[:20])
        lcd_write_line(2, f"AZ:{fmt3(az)} EL:{fmt3(el)}")
        lcd_write_line(3, lcd_center_20("Frequency Ctrl" if freq_mode else "CALIBRATE"))
        time.sleep(0.25)


def primary_send_reply(s: str) -> None:
    global primary_serial
    with primary_lock:
        if primary_serial is not None:
            try:
                primary_serial.write((s + "\r\n").encode())
                primary_serial.flush()
            except Exception as e:
                logPC.add(f"serial write failed: {e}")
    logPC.add(f"<< {s}")


def handle_pc_command(line_raw: str) -> None:
    line = line_raw.strip()
    if not line:
        return

    logPC.add(f">> {line}")
    up = line.upper()

    # GS-232 current position query. SatPC32 polls this heavily, so never
    # answer ERR here. Prefer cached position, then poll rotctld, then fall
    # back to the last cached value or 000/000.
    if up == "C2":
        ok, az, el = current_rot_position(prefer_cached=True)
        primary_send_reply(gs232_position_reply(az, el))
        return

    # Manual/query compatibility commands.
    if up in ("C", "W", "P") or line == "p":
        ok, az, el = current_rot_position(prefer_cached=False)
        primary_send_reply(gs232_position_reply(az, el) if ok else "ERR")
        return

    # GS-232 set command: Waaa eee or Waaaeee
    if len(up) >= 2 and up[0] == "W":
        rest = up[1:].strip()
        a_str = ""
        b_str = ""
        if " " in rest or "	" in rest:
            parts = rest.split()
            if len(parts) >= 2:
                a_str, b_str = parts[0], parts[1]
        elif len(rest) >= 6:
            a_str, b_str = rest[:3], rest[3:6]

        try:
            az = float(a_str)
            el = float(b_str)
        except Exception:
            primary_send_reply("ERR")
            return

        with state.lock:
            state.last_pc_az = az
            state.last_pc_el = el

        ok = rot_set_position(az, el)
        primary_send_reply("OK" if ok else "ERR")
        return

    if up in ("S", "STOP"):
        primary_send_reply("OK")
        return

    primary_send_reply("ERR")


def cat_send_reply(s: str) -> None:
    global secondary_serial
    with secondary_lock:
        if secondary_serial is not None:
            try:
                secondary_serial.write(s.encode())
                secondary_serial.flush()
            except Exception as e:
                logCAT.add(f"serial write failed: {e}")
    logCAT.add(f"<< {s}")


def build_if_response() -> str:
    with state.lock:
        freq = state.vfo_a_hz if state.rx_vfo == 0 else state.vfo_b_hz
        mode = state.mode_code
        rx_vfo = state.rx_vfo
        split = state.split_enabled
        ptt = state.ptt_state
    resp = "IF"
    resp += fmt11(freq)
    resp += "     "
    resp += "+0000"
    resp += "0"
    resp += "0"
    resp += "00"
    resp += "1" if ptt else "0"
    resp += str(mode)
    resp += str(rx_vfo)
    resp += "0"
    resp += "1" if split else "0"
    resp += "0"
    resp += "00"
    resp += "0;"
    return resp


def handle_cat_command(raw_cmd: str) -> None:
    cmd = raw_cmd.strip().upper()
    if not cmd:
        return
    with state.lock:
        state.last_cat_cmd = cmd + ";"
    logCAT.add(f">> {cmd};")
    if cmd == "ID":
        cat_send_reply("ID019;")
        return
    if cmd.startswith("AI"):
        with state.lock:
            if len(cmd) == 2:
                cat_send_reply(f"AI{state.ai_mode};")
                return
            if len(cmd) == 3 and cmd[2].isdigit():
                state.ai_mode = int(cmd[2])
                cat_send_reply(f"AI{state.ai_mode};")
                return
        cat_send_reply("?;")
        return
    if cmd.startswith("PS"):
        cat_send_reply("PS1;")
        return
    if cmd.startswith("FR"):
        with state.lock:
            if len(cmd) == 2:
                cat_send_reply(f"FR{state.rx_vfo};")
                return
            if len(cmd) == 3 and cmd[2] in "01":
                state.rx_vfo = int(cmd[2])
                cat_send_reply(f"FR{state.rx_vfo};")
                return
        cat_send_reply("?;")
        return
    if cmd.startswith("FT"):
        with state.lock:
            if len(cmd) == 2:
                cat_send_reply(f"FT{state.tx_vfo};")
                return
            if len(cmd) == 3 and cmd[2] in "01":
                state.tx_vfo = int(cmd[2])
                state.split_enabled = state.tx_vfo != state.rx_vfo
                cat_send_reply(f"FT{state.tx_vfo};")
                return
        cat_send_reply("?;")
        return
    if cmd.startswith("FA"):
        with state.lock:
            if len(cmd) == 2:
                cat_send_reply(f"FA{fmt11(state.vfo_a_hz)};")
                return
        payload = cmd[2:]
        if len(payload) == 11 and payload.isdigit():
            hz = int(payload)
            with state.lock:
                state.vfo_a_hz = hz
                rx_vfo = state.rx_vfo
            if rx_vfo == 0:
                rig_set_freq(hz)
            cat_send_reply(f"FA{fmt11(hz)};")
            return
        cat_send_reply("?;")
        return
    if cmd.startswith("FB"):
        with state.lock:
            if len(cmd) == 2:
                cat_send_reply(f"FB{fmt11(state.vfo_b_hz)};")
                return
        payload = cmd[2:]
        if len(payload) == 11 and payload.isdigit():
            hz = int(payload)
            with state.lock:
                state.vfo_b_hz = hz
            cat_send_reply(f"FB{fmt11(hz)};")
            return
        cat_send_reply("?;")
        return
    if cmd.startswith("MD"):
        with state.lock:
            if len(cmd) == 2:
                cat_send_reply(f"MD{state.mode_code};")
                return
        if len(cmd) == 3 and cmd[2].isdigit():
            mc = int(cmd[2])
            if mc in (1, 2, 3, 4, 5, 6, 7, 9):
                with state.lock:
                    state.mode_code = mc
                rig_set_mode(mc)
                cat_send_reply(f"MD{mc};")
                return
        cat_send_reply("?;")
        return
    if cmd == "RX":
        rig_set_ptt(False)
        with state.lock:
            state.ptt_state = False
        cat_send_reply("RX;")
        return
    if cmd.startswith("TX"):
        rig_set_ptt(True)
        with state.lock:
            state.ptt_state = True
        if len(cmd) == 2:
            cat_send_reply("TX0;")
            return
        if len(cmd) == 3 and cmd[2] in "01":
            cat_send_reply(f"TX{cmd[2]};")
            return
        cat_send_reply("?;")
        return
    if cmd == "IF":
        cat_send_reply(build_if_response())
        return
    cat_send_reply("?;")


def page_header(title: str) -> str:
    return f"""<!doctype html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>{html_escape(title)}</title>
<style>
body{{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;margin:0;background:#0b0d10;color:#e9eef5}}
.wrap{{max-width:1100px;margin:0 auto;padding:18px}}
.card{{background:#121722;border:1px solid #202a3a;border-radius:14px;padding:14px;margin:12px 0}}
.row{{display:flex;gap:12px;flex-wrap:wrap}}
.col{{flex:1;min-width:280px}}
h1{{font-size:20px;margin:0 0 6px 0}}
h2{{font-size:14px;margin:0 0 10px 0;color:#b8c4d6;font-weight:600}}
a{{color:#8cc4ff;text-decoration:none}} a:hover{{text-decoration:underline}}
input,select{{width:100%;padding:10px;border-radius:10px;border:1px solid #2a3a52;background:#0f1420;color:#e9eef5;box-sizing:border-box}}
button{{padding:10px 12px;border:0;border-radius:10px;background:#2b6cff;color:white;font-weight:700;cursor:pointer}}
button.secondary{{background:#263247}}
.pill{{display:inline-block;padding:4px 10px;border-radius:999px;background:#1a2436;border:1px solid #2a3a52;color:#cfe0ff;font-size:12px}}
.mono{{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;white-space:pre-wrap;overflow:auto;max-height:300px;background:#0f1420;border:1px solid #2a3a52;border-radius:12px;padding:10px}}
.tabs a{{display:inline-block;padding:8px 12px;border-radius:10px;background:#1a2436;border:1px solid #2a3a52;margin-right:8px}}
</style></head><body><div class='wrap'>
<div class='card'><h1>W9KSB Bridge (Pi)</h1>
<div class='row'>
<div class='col'><span class='pill'>First Serial: 57600</span> <span class='pill'>Second Serial: 57600</span> <span class='pill'>Debug: {str(DEBUG)}</span></div>
<div class='col'><a href='/'>Status</a> &nbsp;|&nbsp; <a href='/settings?tab=rotator'>Settings</a> &nbsp;|&nbsp; <a href='/debug'>Debug</a> &nbsp;|&nbsp; <a href='/backup'>Backup / Restore</a></div>
</div></div>"""


def page_footer() -> str:
    return "</div></body></html>"


def settings_tabs(active: str) -> str:
    def tab(name: str, label: str) -> str:
        style = " style='background:#2b6cff;color:white'" if active == name else ""
        return f"<a href='/settings?tab={name}'{style}>{label}</a>"
    return "<div class='card tabs'>" + tab("rotator", "Rotator") + tab("sdr", "SDR") + tab("hardware", "Hardware") + "</div>"


@app.route("/")
def handle_root():
    with state.lock:
        last_pc_az = state.last_pc_az
        last_pc_el = state.last_pc_el
        last_rot_az = state.last_rot_az
        last_rot_el = state.last_rot_el
        last_cat_cmd = state.last_cat_cmd
        last_rig_cmd = state.last_rig_cmd
        p_open = state.primary_serial_open
        s_open = state.secondary_serial_open
        p_err = state.primary_serial_error
        s_err = state.secondary_serial_error
        enc_status = state.encoder_status
        hid_status = state.hid_status
        hid_q = state.hid_queue_depth
        hid_sent = state.hid_sent
        hid_drop = state.hid_dropped
        hid_last = state.hid_last_key

    body = page_header("Status")
    body += "<div class='card'><h2>Network Status</h2><div class='row'>"
    body += network_status_block()
    body += "</div></div>"

    body += "<div class='card'><h2>Device Details</h2><div class='row'>"
    body += "<div class='col'><div class='mono'><div style='font-size:16px;font-weight:bold'>HID Keyboard</div>Encoder hotkeys used for frequency adjustment or arrow key control.</div></div>"
    body += "<div class='col'><div class='mono'><div style='font-size:16px;font-weight:bold'>First Serial (COM) Port</div>Rotator Control — accepts GS-232 commands from software such as SatPC32 and forwards them to rotctld.</div></div>"
    body += "<div class='col'><div class='mono'><div style='font-size:16px;font-weight:bold'>Second Serial (COM) Port</div>SDR Control — accepts TS-2000 CAT commands and translates them to rigctl / SDR.</div></div>"
    body += "</div><div class='mono' style='margin-top:12px'><b>Note:</b> Windows assigns the actual COM numbers. Both serial ports operate at 57600 baud.</div></div>"

    body += "<div class='card'><h2>Live Status</h2><div class='row'>"
    body += f"<div class='col'><span class='pill'>{html_escape(wifi_status_line())}</span></div>"
    body += f"<div class='col'><span class='pill'>{'ROT OK' if state.rot_connected else 'ROT --'}</span></div>"
    body += f"<div class='col'><span class='pill'>{'SDR OK' if state.rig_connected else 'SDR --'}</span></div>"
    body += "</div><div class='row' style='margin-top:12px'>"
    body += f"<div class='col'><div class='mono'><b>Last from PC</b><br>AZ: {fmt3(last_pc_az)} EL: {fmt3(last_pc_el)}</div></div>"
    body += f"<div class='col'><div class='mono'><b>Last from ROT</b><br>AZ: {fmt3(last_rot_az)} EL: {fmt3(last_rot_el)}</div></div>"
    body += f"<div class='col'><div class='mono'><b>Last CAT / Rigctl</b><br>CAT: {html_escape(last_cat_cmd)}<br>Rigctl: {html_escape(last_rig_cmd)}</div></div>"
    body += "</div><div class='row' style='margin-top:12px'>"
    body += f"<div class='col'><div class='mono'><b>USB Port State</b><br>First serial: {'OPEN' if p_open else 'CLOSED'}<br>Second serial: {'OPEN' if s_open else 'CLOSED'}<br>First err: {html_escape(p_err or '-')}<br>Second err: {html_escape(s_err or '-')}</div></div>"
    body += f"<div class='col'><div class='mono'><b>Encoder State</b><br>{html_escape(enc_status)}</div></div>"
    body += f"<div class='col'><div class='mono'><b>HID State</b><br>Status: {html_escape(hid_status)}<br>Queue: {hid_q}<br>Sent: {hid_sent}<br>Dropped: {hid_drop}<br>Last: {html_escape(hid_last)}</div></div>"
    body += "</div></div>"

    body += """<div class='card'><h2>Quick Set</h2><form method='POST' action='/set'><div class='row'>
<div class='col'>AZ (0-360)<br><input name='az' value='0'></div>
<div class='col'>EL (0-90)<br><input name='el' value='0'></div>
<div class='col' style='display:flex;align-items:flex-end;gap:10px'><button type='submit'>Send</button><a class='pill' href='/settings?tab=rotator'>Settings</a></div>
</div></form></div>"""
    body += page_footer()
    return Response(body, mimetype="text/html")


@app.route("/set", methods=["POST"])
def handle_set():
    try:
        az = float(request.form.get("az", "0"))
        el = float(request.form.get("el", "0"))
    except Exception:
        return Response("missing args", status=400, mimetype="text/plain")
    with state.lock:
        state.last_pc_az = az
        state.last_pc_el = el
    logROT.add(f"WEB set AZ={az:.1f} EL={el:.1f}")
    rot_set_position(az, el)
    return redirect("/", code=303)


@app.route("/settings")
def handle_settings():
    tab = request.args.get("tab", "rotator")
    if tab not in ("rotator", "sdr", "hardware"):
        tab = "rotator"

    body = page_header("Settings")
    body += settings_tabs(tab)

    if tab == "rotator":
        body += "<div class='card'><h2>Rotator Settings</h2><form method='POST' action='/save/rotator'>"
        body += "<div class='row'>"
        body += f"<div class='col'>Enable Rotator<br><select name='roten'><option value='1' {'selected' if cfg.rot_enabled else ''}>Yes</option><option value='0' {'' if cfg.rot_enabled else 'selected'}>No</option></select></div>"
        body += f"<div class='col'>rotctld Host/IP<br><input name='rhost' value='{html_escape(cfg.rot_host)}'></div>"
        body += f"<div class='col'>rotctld Port<br><input name='rport' value='{cfg.rot_port}'></div>"
        body += "</div><div class='row' style='margin-top:10px'>"
        body += f"<div class='col'>Safety Limits<br><select name='safety'><option value='1' {'selected' if cfg.safety_enabled else ''}>On</option><option value='0' {'' if cfg.safety_enabled else 'selected'}>Off</option></select></div>"
        body += f"<div class='col'>First Serial Device<br><input name='primary_serial' value='{html_escape(cfg.primary_serial)}'></div>"
        body += f"<div class='col'>First Serial Baud<br><input name='primary_baud' value='{cfg.primary_baud}'></div>"
        body += "</div><div style='margin-top:12px'><button type='submit'>Save Rotator Settings</button></div></form></div>"

    if tab == "sdr":
        body += "<div class='card'><h2>SDR Settings</h2><form method='POST' action='/save/sdr'>"
        body += "<div class='row'>"
        body += f"<div class='col'>Enable SDR Bridge<br><select name='sdren'><option value='1' {'selected' if cfg.sdr_enabled else ''}>Yes</option><option value='0' {'' if cfg.sdr_enabled else 'selected'}>No</option></select></div>"
        body += f"<div class='col'>rigctl Enabled<br><select name='rigen'><option value='1' {'selected' if cfg.rigctl_enabled else ''}>Yes</option><option value='0' {'' if cfg.rigctl_enabled else 'selected'}>No</option></select></div>"
        body += f"<div class='col'>Second Serial Device<br><input name='secondary_serial' value='{html_escape(cfg.secondary_serial)}'></div>"
        body += "</div><div class='row' style='margin-top:10px'>"
        body += f"<div class='col'>SDR Host/IP<br><input name='shost' value='{html_escape(cfg.sdr_host)}'></div>"
        body += f"<div class='col'>SDR Port<br><input name='sport' value='{cfg.sdr_port}'></div>"
        body += f"<div class='col'>Second Serial Baud<br><input name='secondary_baud' value='{cfg.secondary_baud}'></div>"
        body += "</div><div class='row' style='margin-top:10px'>"
        body += f"<div class='col'>Frequency Poll (ms)<br><input name='spoll' value='{cfg.sdr_poll_ms}'></div>"
        body += f"<div class='col'>Mode Poll (ms)<br><input name='mpoll' value='{cfg.mode_poll_ms}'></div>"
        body += f"<div class='col'>PTT Poll (ms)<br><input name='ppoll' value='{cfg.ptt_poll_ms}'></div>"
        body += "</div><div style='margin-top:12px'><button type='submit'>Save SDR Settings</button></div></form></div>"

    if tab == "hardware":
        with state.lock:
            raw_a = state.encoder_raw_a
            raw_b = state.encoder_raw_b
            raw_sw = state.encoder_raw_sw
            last_dir = state.encoder_last_dir
            pos = state.encoder_position
            enc_status = state.encoder_status
            enc_exc = state.encoder_exceptions
            hid_status = state.hid_status
            hid_q = state.hid_queue_depth
            hid_sent = state.hid_sent
            hid_drop = state.hid_dropped
            hid_last = state.hid_last_key
        body += "<div class='card'><h2>Hardware Settings</h2><form method='POST' action='/save/hardware'>"
        body += "<div class='row'>"
        body += f"<div class='col'>HID Device Path<br><input name='hid_path' value='{html_escape(cfg.hid_path)}'></div>"
        body += f"<div class='col'>LCD Enabled<br><select name='lcd_enabled'><option value='1' {'selected' if cfg.lcd_enabled else ''}>Yes</option><option value='0' {'' if cfg.lcd_enabled else 'selected'}>No</option></select></div>"
        body += f"<div class='col'>LCD I2C Address (hex)<br><input name='lcd_addr' value='0x{cfg.lcd_addr:02X}'></div>"
        body += "</div><div class='row' style='margin-top:10px'>"
        body += f"<div class='col'>Encoder Enabled<br><select name='encoder_enabled'><option value='1' {'selected' if cfg.encoder_enabled else ''}>Yes</option><option value='0' {'' if cfg.encoder_enabled else 'selected'}>No</option></select></div>"
        body += f"<div class='col'>Encoder A GPIO<br><input name='enc_a_pin' value='{cfg.enc_a_pin}'></div>"
        body += f"<div class='col'>Encoder B GPIO<br><input name='enc_b_pin' value='{cfg.enc_b_pin}'></div>"
        body += "</div><div class='row' style='margin-top:10px'>"
        body += f"<div class='col'>Encoder SW GPIO<br><input name='enc_sw_pin' value='{cfg.enc_sw_pin}'></div>"
        body += f"<div class='col'>Encoder Button Key<br><input name='btn_key' value='{html_escape(cfg.btn_key)}'></div>"
        body += "</div><div class='row' style='margin-top:10px'>"
        body += f"<div class='col'><div class='mono'><b>Encoder Raw A</b><br>{html_escape(raw_a)}</div></div>"
        body += f"<div class='col'><div class='mono'><b>Encoder Raw B</b><br>{html_escape(raw_b)}</div></div>"
        body += f"<div class='col'><div class='mono'><b>Encoder Raw SW</b><br>{html_escape(raw_sw)}</div></div>"
        body += "</div><div class='row' style='margin-top:10px'>"
        body += f"<div class='col'><div class='mono'><b>Last Direction</b><br>{html_escape(last_dir)}</div></div>"
        body += f"<div class='col'><div class='mono'><b>Encoder Position</b><br>{pos}</div></div>"
        body += f"<div class='col'><div class='mono'><b>Encoder Status</b><br>{html_escape(enc_status)}<br>Exceptions: {enc_exc}</div></div>"
        body += "</div><div class='row' style='margin-top:10px'>"
        body += f"<div class='col'><div class='mono'><b>HID Status</b><br>{html_escape(hid_status)}</div></div>"
        body += f"<div class='col'><div class='mono'><b>HID Queue/Sent</b><br>Queue: {hid_q}<br>Sent: {hid_sent}</div></div>"
        body += f"<div class='col'><div class='mono'><b>HID Dropped/Last</b><br>Dropped: {hid_drop}<br>Last: {html_escape(hid_last)}</div></div>"
        body += "</div><div style='margin-top:12px'><button type='submit'>Save Hardware Settings</button></div></form></div>"

    body += page_footer()
    return Response(body, mimetype="text/html")


@app.route("/save/rotator", methods=["POST"])
def save_rotator():
    cfg.rot_enabled = request.form.get("roten", "1") != "0"
    cfg.rot_host = request.form.get("rhost", cfg.rot_host).strip() or cfg.rot_host
    cfg.rot_port = int(request.form.get("rport", cfg.rot_port))
    cfg.safety_enabled = request.form.get("safety", "1") != "0"
    cfg.primary_serial = request.form.get("primary_serial", cfg.primary_serial).strip() or cfg.primary_serial
    cfg.primary_baud = int(request.form.get("primary_baud", cfg.primary_baud))
    save_config()
    rot_disconnect()
    logROT.add("Saved rotator settings")
    return redirect("/settings?tab=rotator", code=303)


@app.route("/save/sdr", methods=["POST"])
def save_sdr():
    cfg.sdr_enabled = request.form.get("sdren", "1") != "0"
    cfg.rigctl_enabled = request.form.get("rigen", "1") != "0"
    cfg.secondary_serial = request.form.get("secondary_serial", cfg.secondary_serial).strip() or cfg.secondary_serial
    cfg.sdr_host = request.form.get("shost", cfg.sdr_host).strip() or cfg.sdr_host
    cfg.sdr_port = int(request.form.get("sport", cfg.sdr_port))
    cfg.secondary_baud = int(request.form.get("secondary_baud", cfg.secondary_baud))
    cfg.sdr_poll_ms = int(request.form.get("spoll", cfg.sdr_poll_ms))
    cfg.mode_poll_ms = int(request.form.get("mpoll", cfg.mode_poll_ms))
    cfg.ptt_poll_ms = int(request.form.get("ppoll", cfg.ptt_poll_ms))
    save_config()
    rig_disconnect()
    logSDR.add("Saved SDR settings")
    return redirect("/settings?tab=sdr", code=303)


@app.route("/save/hardware", methods=["POST"])
def save_hardware():
    cfg.hid_path = request.form.get("hid_path", cfg.hid_path).strip() or cfg.hid_path
    cfg.lcd_enabled = request.form.get("lcd_enabled", "1") != "0"
    cfg.encoder_enabled = request.form.get("encoder_enabled", "1") != "0"
    try:
        cfg.lcd_addr = int(request.form.get("lcd_addr", f"0x{cfg.lcd_addr:02X}"), 16)
    except Exception:
        pass
    cfg.enc_a_pin = int(request.form.get("enc_a_pin", cfg.enc_a_pin))
    cfg.enc_b_pin = int(request.form.get("enc_b_pin", cfg.enc_b_pin))
    cfg.enc_sw_pin = int(request.form.get("enc_sw_pin", cfg.enc_sw_pin))
    cfg.btn_key = request.form.get("btn_key", cfg.btn_key).strip() or cfg.btn_key
    save_config()
    logSYS.add("Saved hardware settings")
    return redirect("/settings?tab=hardware", code=303)


@app.route("/backup")
def handle_backup():
    body = page_header("Backup / Restore")
    body += "<div class='card'><h2>Backup / Restore</h2><div class='mono'>Coming soon</div></div>"
    body += page_footer()
    return Response(body, mimetype="text/html")


@app.route("/debug")
def handle_debug():
    body = page_header("Debug")
    body += "<div class='card'><h2>Debug Console</h2><div class='row'>"

    def pane(pid, title, endpoint):
        return (
            "<div class='col'>"
            f"<b>{html_escape(title)}</b>"
            f"<div style='display:flex;gap:8px;margin:8px 0'>"
            f"<button class='secondary' type='button' onclick=\"refreshLog('{pid}','{endpoint}')\">Refresh</button>"
            f"<button type='button' onclick=\"toggleAuto('{pid}','{endpoint}')\" id='btn_{pid}'>Start</button>"
            f"<select id='sel_{pid}' onchange=\"if(timers['{pid}']){{toggleAuto('{pid}','{endpoint}');toggleAuto('{pid}','{endpoint}');}}\">"
            "<option value='250'>250ms</option><option value='500' selected>500ms</option><option value='1000'>1000ms</option><option value='2000'>2000ms</option>"
            "</select></div>"
            f"<div class='mono' id='{pid}'>click Refresh</div></div>"
        )

    body += pane("rot", "ROT Debug", "/log/rot")
    body += pane("pc", "PC Debug", "/log/pc")
    body += pane("cat", "CAT Debug", "/log/cat")
    body += pane("sdr", "SDR Debug", "/log/sdr")
    body += pane("enc", "ENC Debug", "/log/enc")
    body += pane("sys", "SYS Debug", "/log/sys")
    body += "</div></div>"
    body += """<script>
const timers={};
async function refreshLog(id,ep){
  try{
    const r=await fetch(ep+'?lines=40',{cache:'no-store'});
    const t=await r.text();
    document.getElementById(id).textContent=t||'(empty)';
  }catch(e){document.getElementById(id).textContent='(error) '+e;}
}
function toggleAuto(id,ep){
  const btn=document.getElementById('btn_'+id);
  const sel=document.getElementById('sel_'+id);
  const ms=parseInt(sel.value||'500',10);
  if(timers[id]){clearInterval(timers[id]);timers[id]=null;btn.textContent='Start';btn.classList.remove('secondary');return;}
  refreshLog(id,ep);
  timers[id]=setInterval(()=>refreshLog(id,ep),ms);
  btn.textContent='Stop';btn.classList.add('secondary');
}
window.addEventListener('beforeunload',()=>{for(const k in timers){if(timers[k])clearInterval(timers[k]);}});
</script>"""
    body += page_footer()
    return Response(body, mimetype="text/html")


@app.route("/log/pc")
def log_pc():
    return Response(logPC.get(int(request.args.get("lines", 40))), mimetype="text/plain")


@app.route("/log/rot")
def log_rot():
    return Response(logROT.get(int(request.args.get("lines", 40))), mimetype="text/plain")


@app.route("/log/cat")
def log_cat():
    return Response(logCAT.get(int(request.args.get("lines", 40))), mimetype="text/plain")


@app.route("/log/sdr")
def log_sdr():
    return Response(logSDR.get(int(request.args.get("lines", 40))), mimetype="text/plain")


@app.route("/log/enc")
def log_enc():
    return Response(logENC.get(int(request.args.get("lines", 40))), mimetype="text/plain")


@app.route("/log/sys")
def log_sys():
    return Response(logSYS.get(int(request.args.get("lines", 40))), mimetype="text/plain")


def primary_serial_worker():
    global primary_serial
    while not stop_event.is_set():
        if serial is None:
            logSYS.add("pyserial not installed")
            time.sleep(2)
            continue

        buf = ""
        local_ser = None
        try:
            local_ser = open_usb_serial(cfg.primary_serial, cfg.primary_baud)
            with primary_lock:
                primary_serial = local_ser
            with state.lock:
                state.primary_serial_open = True
                state.primary_serial_error = ""
            logSYS.add(f"Primary serial opened: {cfg.primary_serial} @ {cfg.primary_baud}")

            while not stop_event.is_set():
                ch = local_ser.read(1)
                if not ch:
                    continue
                c = ch.decode(errors="ignore")
                if c in ("\r", "\n"):
                    if buf:
                        handle_pc_command(buf)
                        buf = ""
                else:
                    if len(buf) < 180:
                        buf += c
        except Exception as e:
            with state.lock:
                state.primary_serial_open = False
                state.primary_serial_error = str(e)
            logSYS.add(f"Primary serial error: {e}")
            time.sleep(SERIAL_REOPEN_DELAY)
        finally:
            with primary_lock:
                if primary_serial is not None:
                    try:
                        primary_serial.close()
                    except Exception:
                        pass
                    primary_serial = None
                elif local_ser is not None:
                    try:
                        local_ser.close()
                    except Exception:
                        pass
            with state.lock:
                state.primary_serial_open = False


def secondary_serial_worker():
    global secondary_serial
    while not stop_event.is_set():
        if serial is None:
            time.sleep(2)
            continue

        buf = ""
        local_ser = None
        try:
            local_ser = open_usb_serial(cfg.secondary_serial, cfg.secondary_baud)
            with secondary_lock:
                secondary_serial = local_ser
            with state.lock:
                state.secondary_serial_open = True
                state.secondary_serial_error = ""
            logSYS.add(f"Secondary serial opened: {cfg.secondary_serial} @ {cfg.secondary_baud}")

            while not stop_event.is_set():
                ch = local_ser.read(1)
                if not ch:
                    continue
                c = ch.decode(errors="ignore")
                if c in ("\r", "\n"):
                    continue
                if c == ";":
                    if buf:
                        handle_cat_command(buf)
                        buf = ""
                else:
                    if len(buf) < 200:
                        buf += c
        except Exception as e:
            with state.lock:
                state.secondary_serial_open = False
                state.secondary_serial_error = str(e)
            logSYS.add(f"Secondary serial error: {e}")
            time.sleep(SERIAL_REOPEN_DELAY)
        finally:
            with secondary_lock:
                if secondary_serial is not None:
                    try:
                        secondary_serial.close()
                    except Exception:
                        pass
                    secondary_serial = None
                elif local_ser is not None:
                    try:
                        local_ser.close()
                    except Exception:
                        pass
            with state.lock:
                state.secondary_serial_open = False


def rig_poll_worker():
    last_freq = 0.0
    last_mode = 0.0
    last_ptt = 0.0
    while not stop_event.is_set():
        now = time.time()
        if cfg.sdr_enabled and cfg.rigctl_enabled:
            if now - last_freq >= cfg.sdr_poll_ms / 1000.0:
                freq = rig_get_freq()
                if freq is not None:
                    with state.lock:
                        if state.rx_vfo == 0:
                            state.vfo_a_hz = freq
                        else:
                            state.vfo_b_hz = freq
                        state.rig_connected = True
                last_freq = now
            if now - last_mode >= cfg.mode_poll_ms / 1000.0:
                mode = rig_get_mode()
                if mode is not None:
                    with state.lock:
                        state.mode_code = mode
                        state.rig_connected = True
                last_mode = now
            if now - last_ptt >= cfg.ptt_poll_ms / 1000.0:
                ptt = rig_get_ptt()
                if ptt is not None:
                    with state.lock:
                        state.ptt_state = ptt
                        state.rig_connected = True
                last_ptt = now
        time.sleep(0.1)


def start_threads():
    threading.Thread(target=network_worker, daemon=True).start()
    threading.Thread(target=primary_serial_worker, daemon=True).start()
    threading.Thread(target=secondary_serial_worker, daemon=True).start()
    threading.Thread(target=rig_poll_worker, daemon=True).start()
    threading.Thread(target=lcd_worker, daemon=True).start()
    threading.Thread(target=hid_worker, daemon=True).start()
    threading.Thread(target=encoder_worker, daemon=True).start()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--debug", action="store_true", help="Enable advanced debug logging")
    return p.parse_args()


def main():
    global DEBUG
    args = parse_args()
    DEBUG = args.debug
    if DEBUG:
        print("Advanced debug enabled", flush=True)
    load_config()
    ensure_app_dir()
    init_lcd()
    start_threads()
    logSYS.add("Controller app started")
    app.run(host="0.0.0.0", port=80, debug=False, threaded=True)


if __name__ == "__main__":
    main()
