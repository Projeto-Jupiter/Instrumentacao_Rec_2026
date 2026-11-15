"""
app.py - desktop window for the load cell reader (PySide6 + pyqtgraph).

    python app.py

Needs core.py next to it. Pick the board's COM port (or "Simulator" to try the
app without hardware) and press Connect.
"""
import bisect
import datetime as dt
import sys
import time

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

import core

pg.setConfigOptions(antialias=True, background="w", foreground="k")

SIM_KEY = "__SIM__"


class CalibrationDialog(QtWidgets.QDialog):
    """Two-step wizard: zero with an empty cell, then measure a known mass."""

    def __init__(self, ctl, parent=None):
        super().__init__(parent)
        self.ctl = ctl
        self.stage = "intro"
        self._result = None
        self.setWindowTitle("Calibrate load cell")
        self.setModal(True)
        self.setMinimumWidth(480)

        lay = QtWidgets.QVBoxLayout(self)
        self.text = QtWidgets.QLabel()
        self.text.setWordWrap(True)
        self.text.setTextFormat(QtCore.Qt.TextFormat.RichText)
        lay.addWidget(self.text)

        mass_row = QtWidgets.QHBoxLayout()
        self.grams_label = QtWidgets.QLabel("Known mass:")
        self.grams = QtWidgets.QDoubleSpinBox()
        self.grams.setRange(0.1, 1_000_000.0)
        self.grams.setDecimals(1)
        self.grams.setValue(1000.0)
        self.grams.setSuffix(" g")
        mass_row.addWidget(self.grams_label)
        mass_row.addWidget(self.grams)
        mass_row.addStretch(1)
        lay.addLayout(mass_row)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100)
        lay.addWidget(self.progress)

        buttons = QtWidgets.QHBoxLayout()
        self.copy_btn = QtWidgets.QPushButton("Copy firmware line")
        self.cancel_btn = QtWidgets.QPushButton("Cancel")
        self.primary = QtWidgets.QPushButton()
        self.primary.setDefault(True)
        buttons.addWidget(self.copy_btn)
        buttons.addStretch(1)
        buttons.addWidget(self.cancel_btn)
        buttons.addWidget(self.primary)
        lay.addLayout(buttons)

        self.primary.clicked.connect(self._primary_clicked)
        self.cancel_btn.clicked.connect(self.reject)
        self.copy_btn.clicked.connect(self._copy)
        self._show_stage("intro")

    # ----- stages -----
    def _show_stage(self, stage, message=""):
        self.stage = stage
        name = self.ctl.active_name()
        self.grams_label.setVisible(stage in ("weight", "measuring"))
        self.grams.setVisible(stage in ("weight", "measuring"))
        self.grams.setEnabled(stage == "weight")
        self.progress.setVisible(stage == "measuring")
        self.copy_btn.setVisible(stage == "done")
        self.cancel_btn.setVisible(stage in ("intro", "zeroing", "weight", "measuring"))
        self.cancel_btn.setEnabled(stage in ("intro", "weight"))
        self.primary.setEnabled(stage in ("intro", "weight", "done", "error"))
        if stage == "intro":
            self.text.setText(f"<b>{name}</b><br><br>Step 1 of 2: take <b>all</b> weight off the load "
                              "cell and keep it still, then press <i>Zero</i>.")
            self.primary.setText("Zero")
        elif stage == "zeroing":
            self.text.setText("Zeroing... (about 2 seconds)")
            self.primary.setText("Zero")
        elif stage == "weight":
            self.text.setText("Step 2 of 2: put the known weight on the cell and wait until it stops "
                              "moving. Type its mass, then press <i>Measure</i> (about 12 seconds).")
            self.primary.setText("Measure")
        elif stage == "measuring":
            self.progress.setValue(0)
            self.text.setText("Measuring... keep everything still.")
            self.primary.setText("Measure")
        elif stage == "done":
            self.text.setText(message)
            self.primary.setText("Close")
        elif stage == "error":
            self.text.setText(f"<span style='color:#b00020'><b>Calibration failed</b></span><br><br>{message}")
            self.primary.setText("Close")

    def _primary_clicked(self):
        if self.stage == "intro":
            self.ctl.cal_start()
            self._show_stage("zeroing")
        elif self.stage == "weight":
            self.ctl.cal_measure(self.grams.value())
            self._show_stage("measuring")
        elif self.stage in ("done", "error"):
            self.accept()

    def _copy(self):
        if self._result:
            _, idx, scale, _, name = self._result
            QtWidgets.QApplication.clipboard().setText(core.firmware_line(name, scale))

    def reject(self):
        if self.stage == "measuring":
            return                                  # the board cannot be interrupted now
        if self.stage in ("zeroing", "weight"):
            self.ctl.cal_cancel()
        super().reject()

    # ----- events from the controller -----
    def on_event(self, ev):
        kind = ev[0]
        if kind == "cal_zero_ok" and self.stage == "zeroing":
            self._show_stage("weight")
        elif kind == "cal_progress" and self.stage == "measuring":
            self.progress.setValue(ev[1])
        elif kind == "cal_result":
            self._result = ev
            _, idx, scale, pp, name = ev
            noise_g = pp / abs(scale) if scale else 0.0
            self._show_stage(
                "done",
                f"<b>{name}</b><br><br>Scale factor: <b>{scale:.4f} counts/g</b><br>"
                f"Noise during the measurement: {noise_g:.2f} g peak-to-peak<br><br>"
                "The app remembers this value for this load cell and uses it from now on. To make it "
                "permanent in the firmware, press <i>Copy firmware line</i> and replace the matching "
                "line in <code>load_cells[]</code> in main.c.")
        elif kind == "cal_error":
            self._show_stage("error", ev[2])


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Load cell reader")
        self.ctl = core.Controller(core.default_settings_path())
        self.cal_dialog = None
        self._dirty = False
        self._last_draw = 0.0
        self._region_placed = False
        self._markers = []
        self._connect_time = 0.0
        self._warned_no_answer = False
        self._build()
        self.refresh_ports()
        self._sync_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(40)
        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self._refresh_stats)
        self.stats_timer.start(500)

    # ------------------------------------------------------------------ UI
    def _build(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # connection
        row = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(300)
        self.refresh_btn = QtWidgets.QPushButton("Refresh ports")
        self.connect_btn = QtWidgets.QPushButton("Connect")
        row.addWidget(QtWidgets.QLabel("Port:"))
        row.addWidget(self.port_combo, 1)
        row.addWidget(self.refresh_btn)
        row.addWidget(self.connect_btn)
        root.addLayout(row)

        # big readout
        self.force_label = QtWidgets.QLabel("--- N")
        font = QtGui.QFont()
        font.setPointSize(44)
        font.setBold(True)
        self.force_label.setFont(font)
        self.force_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.status_label = QtWidgets.QLabel("Not connected")
        self.status_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        root.addWidget(self.force_label)
        root.addWidget(self.status_label)

        # controls
        ctrl = QtWidgets.QHBoxLayout()
        self.stream_btn = QtWidgets.QPushButton("Start readings")
        self.tare_btn = QtWidgets.QPushButton("Zero (tare)")
        self.cell_combo = QtWidgets.QComboBox()
        self.cell_combo.setMinimumWidth(220)
        self.cal_btn = QtWidgets.QPushButton("Calibrate...")
        self.interval_spin = QtWidgets.QSpinBox()
        self.interval_spin.setRange(100, 60000)
        self.interval_spin.setSingleStep(100)
        self.interval_spin.setSuffix(" ms")
        self.interval_spin.setValue(self.ctl.interval_ms)
        self.autozero_chk = QtWidgets.QCheckBox("Auto-zero")
        self.marker_btn = QtWidgets.QPushButton("Marker")
        self.rec_btn = QtWidgets.QPushButton("Record CSV")
        self.rec_btn.setCheckable(True)
        ctrl.addWidget(self.stream_btn)
        ctrl.addWidget(self.tare_btn)
        ctrl.addWidget(QtWidgets.QLabel("Load cell:"))
        ctrl.addWidget(self.cell_combo)
        ctrl.addWidget(self.cal_btn)
        ctrl.addWidget(QtWidgets.QLabel("Every:"))
        ctrl.addWidget(self.interval_spin)
        ctrl.addWidget(self.autozero_chk)
        ctrl.addStretch(1)
        ctrl.addWidget(self.marker_btn)
        ctrl.addWidget(self.rec_btn)
        root.addLayout(ctrl)
        self.controls = [self.stream_btn, self.tare_btn, self.cell_combo, self.cal_btn,
                         self.interval_spin, self.autozero_chk, self.marker_btn, self.rec_btn]

        # plot toolbar + plot
        bar = QtWidgets.QHBoxLayout()
        self.follow_chk = QtWidgets.QCheckBox("Follow")
        self.follow_chk.setChecked(True)
        self.window_spin = QtWidgets.QSpinBox()
        self.window_spin.setRange(5, 3600)
        self.window_spin.setValue(60)
        self.window_spin.setSuffix(" s window")
        self.last_btn = QtWidgets.QPushButton("Selection = last 10 s")
        self.clear_btn = QtWidgets.QPushButton("Clear plot")
        bar.addWidget(self.follow_chk)
        bar.addWidget(self.window_spin)
        bar.addWidget(self.last_btn)
        bar.addStretch(1)
        bar.addWidget(self.clear_btn)
        root.addLayout(bar)

        self.plot = pg.PlotWidget()
        self.plot.setLabel("left", "Force (N)")
        self.plot.setLabel("bottom", "Time (s)")
        self.plot.getAxis("left").enableAutoSIPrefix(False)
        self.plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setAutoVisible(y=True)
        self.curve = self.plot.plot([], [], pen=pg.mkPen("#1f77b4", width=2))
        self.region = pg.LinearRegionItem(values=(0, 10))
        self.region.setZValue(10)
        self.plot.addItem(self.region)
        root.addWidget(self.plot, 1)

        # statistics and drag coefficient
        box = QtWidgets.QGroupBox("Selected region: statistics and drag coefficient")
        grid = QtWidgets.QGridLayout(box)
        self.stats_label = QtWidgets.QLabel("Drag the shaded region on the plot over a steady segment.")
        self.speed_spin = QtWidgets.QDoubleSpinBox()
        self.speed_spin.setRange(0.0, 500.0)
        self.speed_spin.setDecimals(2)
        self.speed_spin.setSuffix(" m/s")
        self.rho_spin = QtWidgets.QDoubleSpinBox()
        self.rho_spin.setRange(0.1, 3.0)
        self.rho_spin.setDecimals(3)
        self.rho_spin.setValue(1.225)
        self.rho_spin.setSuffix(" kg/m³")
        self.area_spin = QtWidgets.QDoubleSpinBox()
        self.area_spin.setRange(0.0, 1000.0)
        self.area_spin.setDecimals(4)
        self.area_spin.setValue(1.0)
        self.area_spin.setSuffix(" m²")
        self.cd_label = QtWidgets.QLabel("Cd = -")
        cd_font = QtGui.QFont()
        cd_font.setPointSize(14)
        cd_font.setBold(True)
        self.cd_label.setFont(cd_font)
        grid.addWidget(self.stats_label, 0, 0, 1, 8)
        grid.addWidget(QtWidgets.QLabel("Air speed:"), 1, 0)
        grid.addWidget(self.speed_spin, 1, 1)
        grid.addWidget(QtWidgets.QLabel("Air density:"), 1, 2)
        grid.addWidget(self.rho_spin, 1, 3)
        grid.addWidget(QtWidgets.QLabel("Reference area:"), 1, 4)
        grid.addWidget(self.area_spin, 1, 5)
        grid.addWidget(self.cd_label, 1, 7)
        root.addWidget(box)

        # simulator controls (only shown with the simulator)
        self.sim_box = QtWidgets.QGroupBox("Simulator: the simulated world")
        sim = QtWidgets.QHBoxLayout(self.sim_box)
        self.sim_weight = QtWidgets.QDoubleSpinBox()
        self.sim_weight.setRange(0.0, 50000.0)
        self.sim_weight.setSingleStep(50.0)
        self.sim_weight.setDecimals(1)
        self.sim_weight.setSuffix(" g on the cell")
        self.sim_scale = QtWidgets.QDoubleSpinBox()
        self.sim_scale.setRange(0.01, 1000.0)
        self.sim_scale.setDecimals(3)
        self.sim_scale.setValue(9.009)
        self.sim_scale.setSuffix(" counts/g (cell plugged in)")
        self.sim_sat = QtWidgets.QCheckBox("Saturate the HX711 input")
        sim.addWidget(self.sim_weight)
        sim.addWidget(self.sim_scale)
        sim.addWidget(self.sim_sat)
        sim.addStretch(1)
        root.addWidget(self.sim_box)

        # log
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFixedHeight(90)
        root.addWidget(self.log)

        # wiring
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.toggle_connect)
        self.stream_btn.clicked.connect(self.on_stream)
        self.tare_btn.clicked.connect(self.on_tare)
        self.cell_combo.activated.connect(self.on_cell)
        self.cal_btn.clicked.connect(self.on_calibrate)
        self.interval_spin.editingFinished.connect(self.on_interval)
        self.autozero_chk.clicked.connect(self.on_autozero)
        self.marker_btn.clicked.connect(self.on_marker)
        self.rec_btn.clicked.connect(self.on_record)
        self.last_btn.clicked.connect(self.on_last_selection)
        self.clear_btn.clicked.connect(self.on_clear)
        self.region.sigRegionChanged.connect(self._refresh_stats)
        self.speed_spin.valueChanged.connect(self._refresh_stats)
        self.rho_spin.valueChanged.connect(self._refresh_stats)
        self.area_spin.valueChanged.connect(self._refresh_stats)
        self.sim_weight.valueChanged.connect(self.on_sim_changed)
        self.sim_scale.valueChanged.connect(self.on_sim_changed)
        self.sim_sat.clicked.connect(self.on_sim_changed)

    # ------------------------------------------------------------ helpers
    def log_msg(self, text):
        self.log.appendPlainText(f"{dt.datetime.now():%H:%M:%S}  {text}")

    def _sim(self):
        return self.ctl.dev if isinstance(self.ctl.dev, core.FakeDevice) else None

    def _sync_ui(self):
        c = self.ctl
        online = c.dev is not None and c.connected
        self.connect_btn.setText("Disconnect" if c.dev is not None else "Connect")
        self.port_combo.setEnabled(c.dev is None)
        self.refresh_btn.setEnabled(c.dev is None)
        for w in self.controls:
            w.setEnabled(online and self.cal_dialog is None)

        self.stream_btn.setText("Stop readings" if c.streaming else "Start readings")

        self.cell_combo.blockSignals(True)
        self.cell_combo.clear()
        for cell in c.cells:
            factor = f"{cell['scale']:.4f} counts/g" if cell["scale"] else "not calibrated"
            self.cell_combo.addItem(f"{cell['idx']}: {cell['name']}  ({factor})")
            if cell["idx"] == c.active:
                self.cell_combo.setCurrentIndex(self.cell_combo.count() - 1)
        self.cell_combo.blockSignals(False)

        self.interval_spin.blockSignals(True)
        self.interval_spin.setValue(c.interval_ms)
        self.interval_spin.blockSignals(False)
        self.autozero_chk.setChecked(c.autozero)
        self.rec_btn.setChecked(c.logger.recording)
        self.sim_box.setVisible(self._sim() is not None)

        if c.dev is None:
            self.status_label.setText("Not connected")
        elif not c.connected:
            self.status_label.setText(f"Connecting to {c.dev.name}...")
        else:
            factor = f"{c.scale:.4f} counts/g" if c.scale else "NOT CALIBRATED (press Calibrate...)"
            self.status_label.setText(f"Connected to {c.dev.name}, firmware {c.fw} | "
                                      f"{c.active_name()}: {factor}")

    # -------------------------------------------------------- connection
    def refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItem("Simulator (no hardware)", SIM_KEY)
        ports = core.list_serial_ports()
        last = self.ctl.settings.get("last_port")
        choice = None
        for device, label, is_stm in ports:
            self.port_combo.addItem(label, device)
            if device == last or (choice is None and is_stm and last in (None, SIM_KEY)):
                choice = self.port_combo.count() - 1
        if choice is None and last == SIM_KEY:
            choice = 0
        if choice is not None:
            self.port_combo.setCurrentIndex(choice)
        elif ports:
            self.port_combo.setCurrentIndex(1)

    def toggle_connect(self):
        if self.ctl.dev is not None:
            self.ctl.disconnect()
            self.force_label.setText("--- N")
            self.force_label.setStyleSheet("")
            self._sync_ui()
            return
        key = self.port_combo.currentData()
        try:
            dev = core.FakeDevice() if key == SIM_KEY else core.SerialDevice(key)
        except ImportError:
            QtWidgets.QMessageBox.critical(self, "pyserial missing",
                                           "Install it with:  pip install pyserial")
            return
        except Exception as e:
            QtWidgets.QMessageBox.critical(
                self, "Cannot open the port",
                f"{key}\n\n{e}\n\nIs another program (for example Tera Term) using this port?")
            return
        self.ctl.settings["last_port"] = key
        self.ctl._save()
        self.on_clear()
        self.ctl.connect(dev)
        self._connect_time = time.monotonic()
        self._warned_no_answer = False
        self.log_msg(f"opened {dev.name}")
        self._sync_ui()

    # ------------------------------------------------------------- slots
    def on_stream(self):
        self.ctl.set_stream(not self.ctl.streaming)

    def on_tare(self):
        self.status_label.setText("Zeroing... keep the load cell unloaded and still.")
        self.ctl.tare()

    def on_cell(self, index):
        if index < 0 or index >= len(self.ctl.cells):
            return
        n = self.ctl.cells[index]["idx"]
        if n == self.ctl.active:
            return
        answer = QtWidgets.QMessageBox.question(
            self, "Change load cell",
            f"Selecting load cell {n} also zeroes it.\nMake sure it is connected and unloaded.",
            QtWidgets.QMessageBox.StandardButton.Ok | QtWidgets.QMessageBox.StandardButton.Cancel)
        if answer != QtWidgets.QMessageBox.StandardButton.Ok:
            self._sync_ui()
            return
        self.status_label.setText("Zeroing the new load cell...")
        self.ctl.select_cell(n)

    def on_calibrate(self):
        if self.cal_dialog is not None:
            return
        self.cal_dialog = CalibrationDialog(self.ctl, self)
        self.cal_dialog.finished.connect(self._cal_closed)
        self.cal_dialog.show()
        self._sync_ui()

    def _cal_closed(self, *_):
        self.cal_dialog = None
        self._sync_ui()

    def on_interval(self):
        if self.interval_spin.value() != self.ctl.interval_ms:
            self.ctl.set_interval(self.interval_spin.value())

    def on_autozero(self, checked):
        self.ctl.set_autozero(bool(checked))

    def on_marker(self):
        text, ok = QtWidgets.QInputDialog.getText(self, "Marker", "Label (optional):")
        if not ok:
            return
        t = self.ctl.add_marker(text.strip())
        line = pg.InfiniteLine(pos=t, angle=90,
                               pen=pg.mkPen("#d62728", width=1, style=QtCore.Qt.PenStyle.DashLine),
                               label=text.strip() or "marker",
                               labelOpts={"position": 0.9, "color": (200, 0, 0), "movable": True})
        self.plot.addItem(line)
        self._markers.append(line)

    def on_record(self, checked):
        if checked:
            default = f"loadcell_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save readings as CSV", default,
                                                            "CSV files (*.csv)")
            if not path:
                self.rec_btn.setChecked(False)
                return
            try:
                self.ctl.record_start(path)
            except OSError as e:
                QtWidgets.QMessageBox.critical(self, "Cannot write the file", str(e))
                self.rec_btn.setChecked(False)
                return
            self.log_msg(f"recording to {path} (start the readings if they are off)")
        else:
            rows = self.ctl.logger.rows
            self.ctl.record_stop()
            self.log_msg(f"recording stopped, {rows} rows")

    def on_last_selection(self):
        ts = self.ctl.times
        if ts:
            self.region.setRegion((max(ts[0], ts[-1] - 10.0), ts[-1]))

    def on_clear(self):
        self.ctl.clear_samples()
        for line in self._markers:
            self.plot.removeItem(line)
        self._markers.clear()
        self._region_placed = False
        self.curve.setData([], [])
        self._refresh_stats()

    def on_sim_changed(self, *_):
        sim = self._sim()
        if sim is not None:
            sim.set_weight(self.sim_weight.value())
            sim.set_true_scale(self.sim_scale.value())
            sim.set_saturated(self.sim_sat.isChecked())

    # ----------------------------------------------------------- events
    def _tick(self):
        for ev in self.ctl.process():
            self._on_event(ev)
            if self.cal_dialog is not None:
                self.cal_dialog.on_event(ev)
        c = self.ctl
        if (c.dev is not None and not c.connected and not self._warned_no_answer
                and time.monotonic() - self._connect_time > 5.0):
            self._warned_no_answer = True
            self.status_label.setText("No answer from the board. Wrong port, or old firmware?")
            self.log_msg("no answer to PING after 5 s")
        now = time.monotonic()
        if self._dirty and now - self._last_draw > 0.05:
            self._dirty = False
            self._last_draw = now
            self._redraw()

    def _on_event(self, ev):
        kind = ev[0]
        if kind == "sample":
            self.force_label.setText(f"{ev[2]:.3f} N")
            self.force_label.setStyleSheet("")
            self._dirty = True
        elif kind in ("connected", "state", "cells", "tared", "saved"):
            if kind == "connected":
                self.log_msg(f"board answered, firmware {ev[1]}")
            if kind == "saved":
                self.log_msg("calibration saved in the app")
            self._sync_ui()
        elif kind == "warn":
            self.log_msg("warning: " + ev[1])
            if ev[1].startswith("SATURATED"):
                self.force_label.setText("SATURATED")
                self.force_label.setStyleSheet("color: #b00020;")
            elif ev[1].startswith("UNCALIBRATED"):
                self.status_label.setText("This load cell has no scale factor yet: press Calibrate...")
        elif kind == "error":
            self.log_msg(ev[3])
            self._sync_ui()
            QtWidgets.QMessageBox.warning(self, "Load cell reader", ev[3])
        elif kind == "cal_error":
            self.log_msg("calibration failed: " + ev[2])
        elif kind == "cal_result":
            self.log_msg(f"calibration result: {ev[4]} = {ev[2]:.4f} counts/g")
        elif kind == "lost":
            self.log_msg("connection lost: " + ev[1])
            self.ctl.disconnect()
            self.force_label.setText("--- N")
            self._sync_ui()
            QtWidgets.QMessageBox.warning(self, "Connection lost",
                                          f"The board was disconnected.\n\n{ev[1]}")
        elif kind == "log":
            self.log_msg(ev[1])

    # ------------------------------------------------------------- plot
    def _redraw(self):
        ts, vs = self.ctl.times, self.ctl.values
        if not ts:
            self.curve.setData([], [])
            return
        if self.follow_chk.isChecked():
            t_end = ts[-1]
            t_start = t_end - self.window_spin.value()
            i = bisect.bisect_left(ts, t_start)
            self.curve.setData(np.asarray(ts[i:]), np.asarray(vs[i:]))
            self.plot.setXRange(t_start, max(t_end, t_start + 1.0), padding=0)
        else:
            step = max(1, len(ts) // 20000)
            self.curve.setData(np.asarray(ts[::step]), np.asarray(vs[::step]))
        if not self._region_placed and len(ts) >= 2:
            self._region_placed = True
            self.region.setRegion((max(ts[0], ts[-1] - 10.0), ts[-1]))

    def _refresh_stats(self, *_):
        lo, hi = self.region.getRegion()
        st = core.window_stats(self.ctl.times, self.ctl.values, lo, hi)
        if st is None:
            self.stats_label.setText("No samples in the selected region. Drag it over the plotted data.")
            self.cd_label.setText("Cd = -")
            return
        n, mean, std, vmin, vmax = st
        self.stats_label.setText(
            f"Region of {abs(hi - lo):.1f} s, {n} samples: mean {mean:.3f} N, "
            f"std {std:.3f} N, min {vmin:.3f} N, max {vmax:.3f} N")
        cd = core.drag_coefficient(mean, self.speed_spin.value(), self.rho_spin.value(),
                                   self.area_spin.value())
        self.cd_label.setText("Cd = -" if cd is None else f"Cd = {cd:.3f}")

    # ------------------------------------------------------------ close
    def closeEvent(self, event):
        self.ctl.disconnect()
        self.ctl._save()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.resize(1100, 860)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
