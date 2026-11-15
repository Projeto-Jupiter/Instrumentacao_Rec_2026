"""
core.py - everything the load cell app needs except the Qt window.

  * protocol   : parse the lines the STM32 sends (F,... OK,... ERR,... W,... CELL,...)
  * devices    : SerialDevice (real board), FakeDevice (pure-Python copy of the
                 firmware, to develop and demo the app without hardware)
  * Controller : state of the connection, command queue, calibration flow,
                 saved calibrations, CSV recording, statistics

Nothing in here imports Qt, so it can be tested without a screen.
"""
from __future__ import annotations

import bisect
import csv
import datetime as dt
import json
import math
import os
import queue
import random
import statistics
import threading
import time
from collections import deque, namedtuple

BAUD = 115200          # ignored by USB CDC, but pyserial wants a value
GRAVITY = 9.80665      # m/s^2

# --------------------------------------------------------------------------
# Protocol
#
#   board -> PC (one message per line, comma separated)
#     F,<ms>,<newtons>                        a reading
#     OK,<cmd>[,...]  /  ERR,<cmd>,<reason>   answer to a command
#     W,<warning>[,...]                       SATURATED, UNCALIBRATED,<n>, ...
#     CELL,<n>,<name>,<counts_per_g>,<active> one line of the CELLS answer
#     CAL,PROGRESS,<percent>                  while a calibration measures
#     STATUS,<stream>,<ms>,<cell>,<scale>,<autozero>,<raw>,<offset>,<cal_hold>
#     # text                                  human-readable, ignored
#   PC -> board: one command per line, e.g. "STREAM 1", "CAL 500"
# --------------------------------------------------------------------------
Msg = namedtuple("Msg", "kind args raw")


def parse_line(line: str):
    """Turn one received line into a Msg (or None for an empty line)."""
    line = line.strip()
    if not line:
        return None
    if line.startswith("#"):
        return Msg("INFO", (line[1:].strip(),), line)
    parts = line.split(",")
    tag = parts[0].strip().upper()
    try:
        if tag == "F" and len(parts) == 3:
            return Msg("F", (int(parts[1]), float(parts[2])), line)
        if tag in ("OK", "ERR") and len(parts) >= 2:
            return Msg(tag, tuple(p.strip() for p in parts[1:]), line)
        if tag == "W" and len(parts) >= 2:
            return Msg("W", tuple(p.strip() for p in parts[1:]), line)
        if tag == "CELL" and len(parts) == 5:
            return Msg("CELL", (int(parts[1]), parts[2], float(parts[3]), parts[4].strip() == "1"), line)
        if tag == "CAL" and len(parts) == 3 and parts[1] == "PROGRESS":
            return Msg("CAL_PROGRESS", (int(parts[2]),), line)
        if tag == "STATUS" and len(parts) == 9:
            return Msg("STATUS", (parts[1] == "1", int(parts[2]), int(parts[3]), float(parts[4]),
                                  parts[5] == "1", int(parts[6]), int(parts[7]), parts[8] == "1"), line)
    except ValueError:
        pass
    return Msg("RAW", (line,), line)


ERROR_TEXT = {
    "SATURATED": "The HX711 input is saturated (value stuck at its maximum). Check the load cell wiring.",
    "NO_HX711": "The HX711 is not responding. Check power and the DT/SCK wires.",
    "NO_WEIGHT": "No weight detected: the reading did not change. Is the weight on the cell?",
    "ZERO_FIRST": "The zero step was skipped. Start the calibration again.",
    "BAD_ARG": "The firmware did not accept that value.",
    "RANGE": "Value out of range.",
    "ARG": "Missing or invalid argument.",
    "TIMEOUT": "The board did not answer in time.",
    "UNKNOWN": "The firmware does not know that command (is it the new firmware?).",
}


def describe_error(cmd: str, reason: str) -> str:
    return f"{cmd}: {ERROR_TEXT.get(reason, reason)}"


# --------------------------------------------------------------------------
# Devices
# --------------------------------------------------------------------------
class Device:
    """Common interface: send() a command line, poll() the parsed messages."""
    name = "device"

    def __init__(self):
        self._rx: "queue.Queue[Msg]" = queue.Queue()
        self.alive = False

    def send(self, text: str) -> None:
        raise NotImplementedError

    def close(self) -> None:
        self.alive = False

    def poll(self, limit: int = 500):
        out = []
        for _ in range(limit):
            try:
                out.append(self._rx.get_nowait())
            except queue.Empty:
                break
        return out

    def _push_line(self, line: str) -> None:
        msg = parse_line(line)
        if msg is not None:
            self._rx.put(msg)

    def _lost(self, reason: str) -> None:
        if self.alive:
            self.alive = False
            self._rx.put(Msg("LOST", (reason,), ""))


def list_serial_ports():
    """[(device, label, looks_like_stm32)] - needs pyserial."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    out = []
    for p in list_ports.comports():
        out.append((p.device, f"{p.device} - {p.description}", p.vid == 0x0483))
    return sorted(out)


class SerialDevice(Device):
    """The real board on a (virtual) COM port."""

    def __init__(self, port: str, baud: int = BAUD):
        super().__init__()
        import serial  # pip install pyserial
        self.name = port
        self._ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
        self._lock = threading.Lock()
        self.alive = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        buf = bytearray()
        try:
            while self.alive:
                data = self._ser.read(256)
                if not data:
                    continue
                buf.extend(data)
                while True:
                    i = buf.find(b"\n")
                    if i < 0:
                        break
                    line = bytes(buf[:i]).decode("ascii", "replace")
                    del buf[: i + 1]
                    self._push_line(line)
                if len(buf) > 4096:       # garbage without newlines
                    buf.clear()
        except Exception as e:             # cable unplugged, port closed...
            self._lost(str(e))

    def send(self, text: str) -> None:
        try:
            with self._lock:
                self._ser.write((text.strip() + "\n").encode("ascii"))
        except Exception as e:
            self._lost(str(e))

    def close(self) -> None:
        self.alive = False
        try:
            self._thread.join(timeout=0.5)
            self._ser.close()
        except Exception:
            pass


class FakeDevice(Device):
    """
    Pure-Python copy of the firmware's behaviour (same protocol, same timing:
    10 samples/s, TARE ~2 s, calibration ~12 s). Used for "Simulator" in the app
    and in tests. `speed` > 1 runs it faster than real time.

    The "physical world" is controlled with set_weight(), set_true_scale()
    (counts per gram of the cell that is plugged in) and set_saturated().
    """
    FW = "sim"
    TABLE = [("Load cell 1", 9.009), ("Load cell 2", 0.0), ("Load cell 3", 0.0)]

    def __init__(self, speed: float = 1.0, seed: int = 1):
        super().__init__()
        self.name = "Simulator"
        self.speed = float(speed)
        self._rng = random.Random(seed)
        self._cmds: "queue.Queue[str]" = queue.Queue()
        # physical world
        self.weight_g = 0.0
        self.true_scale = 9.009
        self.saturated = False
        self.drift_cps = 0.0
        self._drift_t0 = 0
        self._base = 120000.0
        # firmware state
        self.scales = [s for _, s in self.TABLE]
        self.active = 0
        self.scale = self.scales[0]
        self.offset = 0.0
        self.streaming = False
        self.interval = 2000
        self.autozero = True
        self.cal_hold = False
        self.last_raw = 0
        self._med = deque(maxlen=5)
        self._ema = None
        self._t0 = time.monotonic()
        self._last_print = -10 ** 9
        self.alive = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # --- the "physical world" (called from the GUI / tests) ---
    def set_weight(self, grams: float):
        self.weight_g = float(grams)

    def set_true_scale(self, counts_per_g: float):
        self.true_scale = float(counts_per_g)

    def set_saturated(self, on: bool):
        self.saturated = bool(on)

    def set_drift(self, counts_per_s: float):
        self._drift_t0 = self._tick()
        self.drift_cps = float(counts_per_s)

    # --- Device interface ---
    def send(self, text: str) -> None:
        self._cmds.put(text)

    def close(self) -> None:
        self.alive = False

    # --- internals ---
    def _tick(self) -> int:
        return int((time.monotonic() - self._t0) * 1000 * self.speed)

    def _out(self, line: str) -> None:
        self._push_line(line)

    def _read(self) -> int:
        time.sleep(0.1 / self.speed)              # HX711 at 10 samples/s
        if self.saturated:
            raw = 8388607
        else:
            raw = round(self._base + self.weight_g * self.true_scale
                        + self.drift_cps * (self._tick() - self._drift_t0) / 1000.0 + self._rng.gauss(0, 3.0))
            raw = max(-8388608, min(8388607, raw))
        self.last_raw = raw
        return raw

    def _filter(self, raw: int) -> float:
        self._med.append(raw)
        med = float(sorted(self._med)[len(self._med) // 2])
        self._ema = med if self._ema is None else self._ema + 0.15 * (med - self._ema)
        return self._ema

    def _filter_reset(self):
        self._med.clear()
        self._ema = None

    def _force(self, filtered: float) -> float:
        return (filtered - self.offset) / self.scale * GRAVITY / 1000.0

    def _tare(self) -> str:
        vals = [self._read() for _ in range(20)]
        self.offset = sum(vals) / len(vals)
        self._filter_reset()
        return "SATURATED" if max(vals) >= 8388607 or min(vals) <= -8388608 else ""

    def _apply_scale(self):
        s = self.scales[self.active]
        self.scale = s if s != 0 else 1.0

    @staticmethod
    def _fmt(v: float, d: int = 4) -> str:
        s = f"{abs(v):.{d}f}"
        return ("-" if v < 0 and float(s) != 0 else "") + s

    def _run(self):
        last_hint = -10 ** 9
        got_command = False
        self._tare()                              # the firmware zeroes itself at power-up
        while self.alive:
            if not got_command and self._tick() - last_hint >= 2000:
                last_hint = self._tick()
                self._out(f"# Load cell reader v{self.FW} ready. Type HELP and press Enter.")
            while True:
                try:
                    line = self._cmds.get_nowait()
                except queue.Empty:
                    break
                if line.strip():
                    got_command = True
                    self._command(line.strip().upper().replace(",", " ").split())
            raw = self._read()
            filtered = self._filter(raw)
            f = self._force(filtered)
            if self.autozero and not self.cal_hold and abs(f) < 0.20:
                self.offset += 0.02 * (filtered - self.offset)
                f = self._force(filtered)
            if abs(f) < 0.05:
                f = 0.0
            if self.streaming and self._tick() - self._last_print + 20 >= self.interval:
                self._last_print = self._tick()
                if raw >= 8388607 or raw <= -8388608:
                    self._out("W,SATURATED")
                else:
                    self._out(f"F,{self._last_print},{self._fmt(f, 3)}")

    def _command(self, a):
        cmd, args = a[0], a[1:]
        out = self._out
        try:
            if cmd == "PING":
                out(f"OK,PING,{self.FW}")
            elif cmd in ("HELP", "?"):
                out("# (simulator) commands: PING STREAM INTERVAL TARE AUTOZERO CELLS CELL SCALE CAL STATUS")
                out("OK,HELP")
            elif cmd == "STREAM":
                if len(args) < 1 or args[0] not in ("0", "1"):
                    return out("ERR,STREAM,ARG")
                self.streaming = args[0] == "1"
                if self.streaming:
                    self._last_print = self._tick() - self.interval
                out(f"OK,STREAM,{int(self.streaming)}")
            elif cmd == "INTERVAL":
                v = float(args[0])
                if not 100 <= v <= 60000:
                    return out("ERR,INTERVAL,RANGE")
                self.interval = int(v)
                out(f"OK,INTERVAL,{self.interval}")
            elif cmd == "AUTOZERO":
                if len(args) < 1 or args[0] not in ("0", "1"):
                    return out("ERR,AUTOZERO,ARG")
                self.autozero = args[0] == "1"
                out(f"OK,AUTOZERO,{int(self.autozero)}")
            elif cmd == "TARE":
                bad = self._tare()
                out(f"ERR,TARE,{bad}" if bad else "OK,TARE")
            elif cmd == "CELLS":
                for i, (name, _) in enumerate(self.TABLE):
                    out(f"CELL,{i + 1},{name},{self._fmt(self.scales[i])},{int(i == self.active)}")
                out(f"OK,CELLS,{len(self.TABLE)}")
            elif cmd == "CELL":
                n = int(float(args[0]))
                if not 1 <= n <= len(self.TABLE):
                    return out("ERR,CELL,RANGE")
                self.active = n - 1
                self.cal_hold = False
                self._apply_scale()
                if self.scales[self.active] == 0:
                    out(f"W,UNCALIBRATED,{n}")
                bad = self._tare()
                if bad:
                    return out(f"ERR,CELL,{bad}")
                out(f"OK,CELL,{n},{self.TABLE[self.active][0]},{self._fmt(self.scales[self.active])}")
            elif cmd == "SCALE":
                n, v = int(float(args[0])), float(args[1])
                if not 1 <= n <= len(self.TABLE):
                    return out("ERR,SCALE,RANGE")
                self.scales[n - 1] = v
                if n - 1 == self.active:
                    self._apply_scale()
                    self._filter_reset()
                out(f"OK,SCALE,{n},{self._fmt(v)}")
            elif cmd == "CAL":
                self._cal(args)
            elif cmd == "STATUS":
                out(f"STATUS,{int(self.streaming)},{self.interval},{self.active + 1},{self._fmt(self.scale)},"
                    f"{int(self.autozero)},{self.last_raw},{int(self.offset)},{int(self.cal_hold)}")
            else:
                out(f"ERR,{cmd},UNKNOWN")
        except (IndexError, ValueError):
            out(f"ERR,{cmd},ARG")

    def _cal(self, args):
        out = self._out
        if not args:
            return out("ERR,CAL,ARG")
        if args[0] == "ZERO":
            bad = self._tare()
            if bad:
                self.cal_hold = False
                return out(f"ERR,CAL,{bad}")
            self.cal_hold = True
            return out("OK,CAL,ZERO")
        if args[0] == "CANCEL":
            self.cal_hold = False
            return out("OK,CAL,CANCEL")
        try:
            grams = float(args[0])
        except ValueError:
            return out("ERR,CAL,BAD_ARG")
        if grams <= 0:
            return out("ERR,CAL,BAD_ARG")
        if not self.cal_hold:
            return out("ERR,CAL,ZERO_FIRST")
        self.streaming = False
        self.cal_hold = False
        settle, avg_n = 15, 100
        total, last_pct, vals = settle + avg_n, -5, []
        for i in range(total):
            v = self._read()
            if i >= settle:
                vals.append(v)
            pct = (i + 1) * 100 // total
            if pct >= last_pct + 5 and pct < 100:
                out(f"CAL,PROGRESS,{pct}")
                last_pct = pct
        if max(vals) >= 8388607 or min(vals) <= -8388608:
            return out("ERR,CAL,SATURATED")
        delta = sum(vals) / len(vals) - self.offset
        if abs(delta) < 50:
            return out("ERR,CAL,NO_WEIGHT")
        new = delta / grams
        self.scales[self.active] = new
        self.scale = new
        self._filter_reset()
        out("CAL,PROGRESS,100")
        out(f"OK,CAL,RESULT,{self.active + 1},{self._fmt(new)},{max(vals) - min(vals)}")


# --------------------------------------------------------------------------
# Helpers: settings file, CSV recording, statistics, drag coefficient
# --------------------------------------------------------------------------
def default_settings_path() -> str:
    return os.path.join(os.path.expanduser("~"), ".loadcell_app.json")


def load_settings(path: str) -> dict:
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
            return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_settings(path: str, data: dict) -> None:
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except OSError:
        pass


class CsvLogger:
    """CSV with a few '#' header lines. Read it with pandas.read_csv(path, comment='#')."""
    COLUMNS = ["pc_time", "device_ms", "elapsed_s", "force_N", "cell", "scale_counts_per_g", "marker"]

    def __init__(self):
        self._f = None
        self._w = None
        self.path = None
        self.rows = 0

    @property
    def recording(self) -> bool:
        return self._f is not None

    def start(self, path: str, meta: dict) -> None:
        self.stop()
        self._f = open(path, "w", newline="", encoding="utf-8")
        for k, v in meta.items():
            self._f.write(f"# {k}: {v}\n")
        self._w = csv.writer(self._f)
        self._w.writerow(self.COLUMNS)
        self._f.flush()
        self.path, self.rows = path, 0

    def write(self, device_ms, elapsed_s, force, cell, scale, marker="") -> None:
        if not self._f:
            return
        self._w.writerow([dt.datetime.now().isoformat(timespec="milliseconds"), device_ms,
                          f"{elapsed_s:.3f}", "" if force is None else f"{force:.3f}",
                          cell, f"{scale:.4f}", marker])
        self._f.flush()
        self.rows += 1

    def stop(self) -> None:
        if self._f:
            try:
                self._f.close()
            except OSError:
                pass
        self._f = self._w = None


def window_stats(times, values, t0, t1):
    """(n, mean, std, min, max) of the samples with t0 <= t <= t1, or None."""
    if t1 < t0:
        t0, t1 = t1, t0
    i, j = bisect.bisect_left(times, t0), bisect.bisect_right(times, t1)
    seg = values[i:j]
    if not seg:
        return None
    return (len(seg), statistics.fmean(seg), statistics.pstdev(seg) if len(seg) > 1 else 0.0,
            min(seg), max(seg))


def drag_coefficient(force_n, speed_ms, rho, area_m2):
    """Cd = 2F / (rho v^2 A), or None if an input is unusable."""
    try:
        if speed_ms <= 0 or rho <= 0 or area_m2 <= 0:
            return None
        return 2.0 * force_n / (rho * speed_ms ** 2 * area_m2)
    except (TypeError, ZeroDivisionError):
        return None


def firmware_line(name: str, scale: float) -> str:
    """The line to paste into load_cells[] in main.c."""
    return f'{{ "{name}", {scale:.4f}f }},'


# --------------------------------------------------------------------------
# Controller
# --------------------------------------------------------------------------
class Controller:
    """
    Holds the app state and talks to a Device. Call process() often (the GUI
    does it every 40 ms); it returns a list of events for the GUI to show:

      ("connected", fw)  ("lost", reason)  ("state",)  ("cells",)  ("tared",)
      ("sample", t_s, force)  ("log", text)  ("warn", text)  ("error", cmd, reason, text)
      ("cal_zero_ok",)  ("cal_progress", pct)  ("cal_result", idx, scale, pp, name)
      ("cal_error", reason, text)  ("saved",)
    """
    TIMEOUTS = {"TARE": 10.0, "CELL": 10.0, "CAL": 45.0}
    DEFAULT_TIMEOUT = 4.0
    MAX_SAMPLES = 500_000

    def __init__(self, settings_path: str | None = None):
        self.settings_path = settings_path
        self.settings = load_settings(settings_path) if settings_path else {}
        self.saved_cal: dict = self.settings.setdefault("cal", {})
        self.dev: Device | None = None
        self.connected = False
        self.fw = ""
        self.cells: list[dict] = []          # {idx, name, scale, active}
        self._cells_tmp: list[dict] = []
        self.flash_scale: dict[int, float] = {}
        self.active = 1
        self.scale = 0.0
        self.streaming = False
        self.interval_ms = int(self.settings.get("interval_ms", 200))
        self.autozero = True
        self.times: list[float] = []         # seconds since the first sample
        self.values: list[float] = []
        self.last_force: float | None = None
        self.last_ms = 0
        self._base_ms: int | None = None
        self.logger = CsvLogger()
        self.cal_state = "idle"              # idle | zeroing | ready | measuring
        self._queue: deque = deque()
        self._inflight = None                # (name, sent_at, timeout)
        self._events: list = []

    # ---------- connection ----------
    def connect(self, dev: Device) -> None:
        self.disconnect()
        self.dev = dev
        self.connected = False
        self.cells, self._cells_tmp, self.flash_scale = [], [], {}
        self.streaming = False
        self.cal_state = "idle"
        self._queue.clear()
        self._inflight = None
        self._send("PING")
        self._send("CELLS")
        self._send(f"INTERVAL {self.interval_ms}")
        self._send("STATUS")

    def disconnect(self) -> None:
        self.logger.stop()
        if self.dev:
            self.dev.close()
        self.dev = None
        self.connected = False
        self.streaming = False
        self.cal_state = "idle"
        self._queue.clear()
        self._inflight = None

    # ---------- commands (the GUI calls these) ----------
    def _send(self, cmd: str) -> None:
        self._queue.append(cmd)

    def set_stream(self, on: bool) -> None:
        self._send(f"STREAM {int(on)}")

    def tare(self) -> None:
        self._send("TARE")

    def select_cell(self, n: int) -> None:
        self._send(f"CELL {n}")

    def set_interval(self, ms: int) -> None:
        self.interval_ms = int(ms)
        self.settings["interval_ms"] = self.interval_ms
        self._save()
        self._send(f"INTERVAL {self.interval_ms}")

    def set_autozero(self, on: bool) -> None:
        self._send(f"AUTOZERO {int(on)}")

    def cal_start(self) -> None:
        self.cal_state = "zeroing"
        self._send("CAL ZERO")

    def cal_measure(self, grams: float) -> None:
        if self.cal_state != "ready":
            return
        self.cal_state = "measuring"
        self.streaming = False               # the firmware pauses the stream
        self._events.append(("state",))
        self._send(f"CAL {grams:g}")

    def cal_cancel(self) -> None:
        if self.cal_state in ("zeroing", "ready"):
            self.cal_state = "idle"
            self._send("CAL CANCEL")

    def active_name(self) -> str:
        for c in self.cells:
            if c["idx"] == self.active:
                return c["name"]
        return f"Load cell {self.active}"

    # ---------- recording / markers ----------
    def record_start(self, path: str) -> None:
        self.logger.start(path, {
            "app": "loadcell_app", "started": dt.datetime.now().isoformat(timespec="seconds"),
            "firmware": self.fw, "cell": f"{self.active} ({self.active_name()})",
            "scale_counts_per_g": f"{self.scale:.4f}", "interval_ms": self.interval_ms,
            "unit": "newtons"})

    def record_stop(self) -> None:
        self.logger.stop()

    def clear_samples(self) -> None:
        self.times.clear()
        self.values.clear()
        self._base_ms = None
        self.last_force = None

    def add_marker(self, text: str = "") -> float:
        t = self.times[-1] if self.times else 0.0
        self.logger.write(self.last_ms, t, self.last_force, self.active, self.scale, text or "marker")
        return t

    # ---------- main pump ----------
    def process(self) -> list:
        events, self._events = self._events, []
        if self.dev is None:
            return events
        for msg in self.dev.poll():
            self._handle(msg, events)
        self._pump(events)
        return events

    def _pump(self, events) -> None:
        now = time.monotonic()
        if self._inflight and now - self._inflight[1] > self._inflight[2]:
            name = self._inflight[0]
            self._inflight = None
            events.append(("log", f"timeout waiting for {name}"))
            if name == "CAL":
                self.cal_state = "idle"
                events.append(("cal_error", "TIMEOUT", describe_error("CAL", "TIMEOUT")))
        if self._inflight is None and self._queue and self.dev is not None:
            cmd = self._queue.popleft()
            name = cmd.split()[0].upper()
            self._inflight = (name, now, self.TIMEOUTS.get(name, self.DEFAULT_TIMEOUT))
            self.dev.send(cmd)

    def _done(self, name: str) -> None:
        if self._inflight and self._inflight[0] == name:
            self._inflight = None

    def _save(self) -> None:
        if self.settings_path:
            save_settings(self.settings_path, self.settings)

    # ---------- message handling ----------
    def _handle(self, msg: Msg, events: list) -> None:
        k, a = msg.kind, msg.args
        if k == "F":
            self._on_sample(a[0], a[1], events)
        elif k == "OK":
            self._on_ok(a, events)
        elif k == "ERR":
            cmd, reason = a[0], (a[1] if len(a) > 1 else "")
            self._done(cmd)
            if cmd == "CAL":
                self.cal_state = "idle"
                events.append(("cal_error", reason, describe_error(cmd, reason)))
            else:
                events.append(("error", cmd, reason, describe_error(cmd, reason)))
                if cmd in ("TARE", "CELL"):
                    events.append(("tared",))
        elif k == "W":
            events.append(("warn", ",".join(a)))
        elif k == "CELL":
            self._cells_tmp.append({"idx": a[0], "name": a[1], "scale": a[2], "active": a[3]})
        elif k == "CAL_PROGRESS":
            events.append(("cal_progress", a[0]))
        elif k == "STATUS":
            self.streaming, self.interval_ms = a[0], a[1]
            self.active, self.scale, self.autozero = a[2], a[3], a[4]
            self._done("STATUS")
            events.append(("state",))
        elif k == "LOST":
            self.connected = False
            self.streaming = False
            self.logger.stop()
            events.append(("lost", a[0]))
        else:   # INFO / RAW
            events.append(("log", msg.raw))

    def _on_sample(self, ms: int, force: float, events: list) -> None:
        if not self.streaming:
            self.streaming = True            # we missed the OK: resync
            events.append(("state",))
        if self._base_ms is None:
            self._base_ms = ms
        elif ms < self.last_ms:              # the board was reset: keep time continuous
            self._base_ms = ms - (self.last_ms - self._base_ms)
        self.last_ms = ms
        t = (ms - self._base_ms) / 1000.0
        self.times.append(t)
        self.values.append(force)
        if len(self.times) > self.MAX_SAMPLES:
            del self.times[:100_000], self.values[:100_000]
        self.last_force = force
        self.logger.write(ms, t, force, self.active, self.scale)
        events.append(("sample", t, force))

    def _on_ok(self, a: tuple, events: list) -> None:
        cmd = a[0]
        self._done(cmd)
        if cmd == "PING":
            self.connected = True
            self.fw = a[1] if len(a) > 1 else ""
            events.append(("connected", self.fw))
        elif cmd == "STREAM":
            self.streaming = (a[1] == "1")
            events.append(("state",))
        elif cmd == "INTERVAL":
            self.interval_ms = int(a[1])
            events.append(("state",))
        elif cmd == "AUTOZERO":
            self.autozero = (a[1] == "1")
            events.append(("state",))
        elif cmd == "TARE":
            events.append(("tared",))
        elif cmd == "CELLS":
            self.cells, self._cells_tmp = self._cells_tmp, []
            self._reconcile_saved(events)
            for c in self.cells:
                if c["active"]:
                    self.active, self.scale = c["idx"], c["scale"]
            events.append(("cells",))
        elif cmd == "CELL":
            self.active, self.scale = int(a[1]), float(a[3])
            for c in self.cells:
                c["active"] = (c["idx"] == self.active)
                if c["active"]:
                    c["scale"] = self.scale
            self._reconcile_saved(events, only=self.active)
            events.extend([("cells",), ("tared",)])
        elif cmd == "SCALE":
            idx, val = int(a[1]), float(a[2])
            for c in self.cells:
                if c["idx"] == idx:
                    c["scale"] = val
                    if c["active"]:
                        self.scale = val
            events.append(("cells",))
        elif cmd == "CAL":
            sub = a[1] if len(a) > 1 else ""
            if sub == "ZERO":
                self.cal_state = "ready"
                events.append(("cal_zero_ok",))
            elif sub == "RESULT":
                idx, scale, pp = int(a[2]), float(a[3]), int(a[4])
                self.cal_state = "idle"
                name = next((c["name"] for c in self.cells if c["idx"] == idx), f"Load cell {idx}")
                for c in self.cells:
                    if c["idx"] == idx:
                        c["scale"] = scale
                if idx == self.active:
                    self.scale = scale
                self.saved_cal[str(idx)] = {"scale": scale, "fw_scale": self.flash_scale.get(idx, 0.0)}
                self._save()
                events.extend([("cal_result", idx, scale, pp, name), ("cells",), ("state",), ("saved",)])
            elif sub == "CANCEL":
                self.cal_state = "idle"

    def _reconcile_saved(self, events: list, only: int | None = None) -> None:
        """
        The app remembers calibrations (settings file). The firmware has its own
        table. Rules per cell, with fw = the firmware's current factor:
          fw == saved                  -> nothing to do (board already uses it)
          fw == factor seen at save    -> board was reset: send our saved factor
          anything else                -> main.c was re-flashed with a new factor: drop ours
        """
        for c in self.cells:
            idx = c["idx"]
            if only is not None and idx != only:
                continue
            fw = c["scale"]
            s = self.saved_cal.get(str(idx))
            if s is None:
                self.flash_scale.setdefault(idx, fw)
                continue
            close = lambda x, y: abs(x - y) < 5e-5
            if close(fw, s["scale"]):
                self.flash_scale[idx] = s.get("fw_scale", fw)
            elif close(fw, s.get("fw_scale", 0.0)):
                self.flash_scale[idx] = fw
                self._send(f"SCALE {idx} {s['scale']:.4f}")
                events.append(("log", f"restored saved calibration of cell {idx}: {s['scale']:.4f} counts/g"))
            else:
                del self.saved_cal[str(idx)]
                self.flash_scale[idx] = fw
                self._save()
                events.append(("log", f"firmware factor of cell {idx} changed; dropped the saved one"))
