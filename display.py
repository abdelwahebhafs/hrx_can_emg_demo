import mmap
import os
import struct
import sys
from datetime import datetime

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QFont, QPainter, QPen, QBrush
from PyQt6.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

STATE_IDLE = 0
STATE_RUNNING = 1

STATE_FMT = "<11f8f8BI"
STATE_SIZE = struct.calcsize(STATE_FMT)

CMD_FMT = "<4Bi256s"
CMD_SIZE = struct.calcsize(CMD_FMT)


class ShmReader:
    def __init__(self):
        self._shm = mmap.mmap(-1, STATE_SIZE, tagname="HRX_Demo_State", access=mmap.ACCESS_READ)

    def read(self):
        self._shm.seek(0)
        raw = struct.unpack(STATE_FMT, self._shm.read(STATE_SIZE))
        head = raw[:11]
        emg = raw[11:19]
        flags = raw[19:27]
        counter = raw[27]
        return {
            "pos1_rad": head[0],
            "pos2_rad": head[1],
            "vel1_rads": head[2],
            "vel2_rads": head[3],
            "torque1_Nm": head[4],
            "torque2_Nm": head[5],
            "analog2_1": head[6],
            "analog2_2": head[7],
            "cmd_raw_mNm": head[8],
            "global_time_s": head[9],
            "global_dt_s": head[10],
            "emg": list(emg),
            "app_state": flags[0],
            "can_ok": flags[1],
            "emg_ok": flags[2],
            "joint1_connected": flags[3],
            "joint2_connected": flags[4],
            "analog2_1_seen": flags[5],
            "analog2_2_seen": flags[6],
            "counter": counter,
        }

    def close(self):
        self._shm.close()


class ShmWriter:
    def __init__(self):
        self._shm = mmap.mmap(-1, CMD_SIZE, tagname="HRX_Demo_Cmd", access=mmap.ACCESS_WRITE)

    def send(self, state, folder, cmd_raw):
        folder_bytes = folder.encode("utf-8")[:255].ljust(256, b"\x00")
        self._shm.seek(0)
        self._shm.write(struct.pack(
            CMD_FMT,
            state,
            1,
            0,
            0,
            int(cmd_raw),
            folder_bytes,
        ))

    def close(self):
        self._shm.close()


class SessionDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HRX demo session")
        self.setFixedSize(600, 280)
        self.setStyleSheet("background:#11161f; color:#dce3ef;")

        self.folder = ""
        self.command_raw = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(12)

        title = QLabel("Create demo session")
        title.setStyleSheet("font-size:22px; font-weight:700; color:white;")
        root.addWidget(title)

        subtitle = QLabel("The UI will create one folder for CAN and EMG logs, then open the single-screen live dashboard.")
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet("color:#96a6c0;")
        root.addWidget(subtitle)

        root.addWidget(self._small("Command raw value sent on CAN ID 0x666"))
        self._cmd_spin = QSpinBox()
        self._cmd_spin.setRange(-32768, 32767)
        self._cmd_spin.setValue(0)
        self._cmd_spin.setStyleSheet(self._input_style())
        root.addWidget(self._cmd_spin)

        root.addWidget(self._small("Output folder"))
        folder_row = QHBoxLayout()
        self._path_lbl = QLabel(os.path.expanduser("~"))
        self._path_lbl.setWordWrap(True)
        self._path_lbl.setStyleSheet("background:#18202d; border:1px solid #334055; border-radius:6px; padding:8px;")
        browse = QPushButton("Browse…")
        browse.setStyleSheet(self._button_style("#31445f"))
        browse.clicked.connect(self._browse)
        folder_row.addWidget(self._path_lbl, 1)
        folder_row.addWidget(browse)
        root.addLayout(folder_row)

        create = QPushButton("Start demo")
        create.setStyleSheet(self._button_style("#0d7a57"))
        create.clicked.connect(self._accept)
        root.addStretch(1)
        root.addWidget(create)

    def _small(self, text):
        w = QLabel(text)
        w.setStyleSheet("color:#96a6c0; font-size:12px;")
        return w

    def _input_style(self):
        return (
            "QSpinBox { background:#18202d; border:1px solid #334055; border-radius:6px; "
            "padding:8px; color:white; min-height:22px; }"
        )

    def _button_style(self, bg):
        return (
            f"QPushButton {{ background:{bg}; color:white; border:none; border-radius:6px; "
            "padding:10px 16px; font-weight:700; }}"
            "QPushButton:pressed { background:#243246; }"
        )

    def _browse(self):
        path = QFileDialog.getExistingDirectory(self, "Choose output folder", os.path.expanduser("~"))
        if path:
            self._path_lbl.setText(path)

    def _accept(self):
        base = self._path_lbl.text().strip() or os.path.expanduser("~")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.folder = os.path.join(base, f"hrx_can_emg_demo_{stamp}")
        os.makedirs(self.folder, exist_ok=True)
        self.command_raw = int(self._cmd_spin.value())
        self.accept()


class RobotCanvas(QFrame):
    def __init__(self):
        super().__init__()
        self._state = {}
        self.setMinimumHeight(280)
        self.setStyleSheet("background:#0c1118; border:1px solid #2a3445; border-radius:10px;")

    def update_state(self, state):
        self._state = state
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#0c1118"))

        w = self.width()
        h = self.height()

        title_font = QFont("Arial", 15, weight=QFont.Weight.Bold)
        label_font = QFont("Arial", 11)
        mono_font = QFont("Consolas", 11)

        p.setFont(title_font)
        p.setPen(QColor("#e7edf7"))
        p.drawText(18, 28, "Robot overview")

        left_x = int(w * 0.28)
        right_x = int(w * 0.72)
        top = 60
        bottom = h - 30

        for x, label in ((left_x, "Robot 1"), (right_x, "Robot 2")):
            p.setPen(QPen(QColor("#3e4a5f"), 6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawLine(x, top, x, bottom)
            p.setPen(QColor("#a8b7ce"))
            p.setFont(label_font)
            p.drawText(x - 36, 48, label)

        s = self._state or {}
        pos1 = max(-3.14, min(3.14, s.get("pos1_rad", 0.0)))
        pos2 = max(-3.14, min(3.14, s.get("pos2_rad", 0.0)))
        y1 = int(top + (pos1 + 3.14) / 6.28 * (bottom - top))
        y2 = int(top + (pos2 + 3.14) / 6.28 * (bottom - top))

        self._draw_robot_marker(p, left_x, y1, bool(s.get("joint1_connected", 0)))
        self._draw_robot_marker(p, right_x, y2, bool(s.get("joint2_connected", 0)))

        p.setFont(mono_font)
        p.setPen(QColor("#d9e3f1"))
        p.drawText(24, h - 74, f"Pos1  {s.get('pos1_rad', 0.0): .4f} rad")
        p.drawText(24, h - 52, f"Vel1  {s.get('vel1_rads', 0.0): .4f} rad/s")
        p.drawText(24, h - 30, f"Trq1  {s.get('torque1_Nm', 0.0): .4f} Nm")

        p.drawText(w // 2 + 18, h - 74, f"Pos2  {s.get('pos2_rad', 0.0): .4f} rad")
        p.drawText(w // 2 + 18, h - 52, f"Vel2  {s.get('vel2_rads', 0.0): .4f} rad/s")
        p.drawText(w // 2 + 18, h - 30, f"Trq2  {s.get('torque2_Nm', 0.0): .4f} Nm")

    def _draw_robot_marker(self, painter, x, y, connected):
        color = QColor("#21c47c") if connected else QColor("#d65b72")
        painter.setPen(QPen(color.lighter(130), 2))
        painter.setBrush(QBrush(color))
        painter.drawEllipse(x - 20, y - 20, 40, 40)


class MainWindow(QMainWindow):
    def __init__(self, reader, writer, folder, initial_cmd):
        super().__init__()
        self._reader = reader
        self._writer = writer
        self._folder = folder
        self._last_counter = -1
        self._current_state = STATE_IDLE

        self.setWindowTitle("HRX CAN + EMG demo")
        self.setMinimumSize(1200, 760)
        self.setStyleSheet("background:#10141c; color:#dce3ef;")

        self._build_ui()
        self._cmd_spin.setValue(int(initial_cmd))
        self._send_state(STATE_IDLE)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(16)

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(18, 18, 18, 18)
        main.setSpacing(12)

        top = QHBoxLayout()
        self._state_lbl = QLabel("IDLE")
        self._state_lbl.setStyleSheet(self._badge("#31445f"))
        self._can_lbl = QLabel("CAN ○")
        self._emg_lbl = QLabel("EMG ○")
        self._j1_lbl = QLabel("Robot 1 ○")
        self._j2_lbl = QLabel("Robot 2 ○")
        self._a1_lbl = QLabel("Analog2-1 ○")
        self._a2_lbl = QLabel("Analog2-2 ○")

        top.addWidget(self._state_lbl)
        for w in (self._can_lbl, self._emg_lbl, self._j1_lbl, self._j2_lbl, self._a1_lbl, self._a2_lbl):
            top.addWidget(w)
        top.addStretch(1)

        folder_lbl = QLabel(self._folder)
        folder_lbl.setStyleSheet("color:#8d9cb4; font-size:11px;")
        top.addWidget(folder_lbl)
        main.addLayout(top)

        self._canvas = RobotCanvas()
        main.addWidget(self._canvas)

        middle = QHBoxLayout()
        middle.addWidget(self._build_can_panel(), 2)
        middle.addWidget(self._build_emg_panel(), 3)
        main.addLayout(middle)

        controls = self._build_controls()
        main.addLayout(controls)

    def _build_can_panel(self):
        box = self._panel("Decoded CAN values")
        grid = QGridLayout(box)

        labels = [
            ("pos1", "Position 1 [rad]"),
            ("pos2", "Position 2 [rad]"),
            ("vel1", "Velocity 1 [rad/s]"),
            ("vel2", "Velocity 2 [rad/s]"),
            ("trq1", "Torque 1 [Nm]"),
            ("trq2", "Torque 2 [Nm]"),
            ("a1", "Analog2 1 [raw]"),
            ("a2", "Analog2 2 [raw]"),
            ("cmd", "Command raw [mNm]"),
            ("time", "Global time [s]"),
            ("dt", "Global dt [s]"),
        ]
        self._vals = {}
        for idx, (key, text) in enumerate(labels):
            r = idx
            lbl = QLabel(text)
            lbl.setStyleSheet("color:#97a7bf;")
            val = QLabel("—")
            val.setStyleSheet("color:white; font-size:16px; font-weight:700;")
            self._vals[key] = val
            grid.addWidget(lbl, r, 0)
            grid.addWidget(val, r, 1)
        return box

    def _build_emg_panel(self):
        box = self._panel("Live EMG 0–7")
        grid = QGridLayout(box)
        self._emg_vals = []
        for i in range(8):
            name = QLabel(f"EMG {i}")
            name.setStyleSheet("color:#97a7bf;")
            val = QLabel("0.0000")
            val.setStyleSheet("color:white; font-size:16px; font-weight:700;")
            self._emg_vals.append(val)
            grid.addWidget(name, i, 0)
            grid.addWidget(val, i, 1)
        return box

    def _build_controls(self):
        row = QHBoxLayout()

        self._cmd_spin = QSpinBox()
        self._cmd_spin.setRange(-32768, 32767)
        self._cmd_spin.valueChanged.connect(lambda _v: self._send_state(self._current_state))
        self._cmd_spin.setStyleSheet(
            "QSpinBox { background:#18202d; border:1px solid #334055; border-radius:6px; "
            "padding:8px; color:white; min-width:150px; }"
        )

        start_btn = QPushButton("Start logging")
        stop_btn = QPushButton("Stop logging")
        zero_btn = QPushButton("Set command = 0")
        quit_btn = QPushButton("Exit")

        for btn, bg in (
            (start_btn, "#0d7a57"),
            (stop_btn, "#8f2747"),
            (zero_btn, "#714b13"),
            (quit_btn, "#31445f"),
        ):
            btn.setStyleSheet(
                f"QPushButton {{ background:{bg}; color:white; border:none; border-radius:7px; "
                "padding:12px 16px; font-weight:700; min-width:140px; }}"
                "QPushButton:pressed { background:#243246; }"
            )

        start_btn.clicked.connect(lambda: self._send_state(STATE_RUNNING))
        stop_btn.clicked.connect(lambda: self._send_state(STATE_IDLE))
        zero_btn.clicked.connect(self._zero_command)
        quit_btn.clicked.connect(self._exit)

        row.addWidget(QLabel("Command raw"))
        row.addWidget(self._cmd_spin)
        row.addWidget(start_btn)
        row.addWidget(stop_btn)
        row.addWidget(zero_btn)
        row.addStretch(1)
        row.addWidget(quit_btn)
        return row

    def _panel(self, title):
        box = QGroupBox(title)
        box.setStyleSheet(
            "QGroupBox { border:1px solid #2f3b4f; border-radius:8px; margin-top:8px; padding:10px; font-weight:700; }"
            "QGroupBox::title { subcontrol-origin: margin; left:10px; padding:0 4px; color:#95a6c4; }"
        )
        return box

    def _badge(self, bg):
        return (
            f"background:{bg}; color:white; border-radius:8px; padding:8px 16px; "
            "font-size:16px; font-weight:700;"
        )

    def _send_state(self, state):
        self._writer.send(state=state, folder=self._folder, cmd_raw=self._cmd_spin.value())

    def _zero_command(self):
        self._cmd_spin.setValue(0)
        self._send_state(self._current_state)

    def _poll(self):
        try:
            state = self._reader.read()
        except Exception:
            return

        if state["counter"] == self._last_counter:
            return
        self._last_counter = state["counter"]

        self._canvas.update_state(state)

        app_state = state["app_state"]
        self._current_state = app_state
        self._state_lbl.setText("RUNNING" if app_state == STATE_RUNNING else "IDLE")
        self._state_lbl.setStyleSheet(self._badge("#0d7a57" if app_state == STATE_RUNNING else "#31445f"))

        self._set_indicator(self._can_lbl, "CAN", bool(state["can_ok"]))
        self._set_indicator(self._emg_lbl, "EMG", bool(state["emg_ok"]))
        self._set_indicator(self._j1_lbl, "Robot 1", bool(state["joint1_connected"]))
        self._set_indicator(self._j2_lbl, "Robot 2", bool(state["joint2_connected"]))
        self._set_indicator(self._a1_lbl, "Analog2-1", bool(state["analog2_1_seen"]))
        self._set_indicator(self._a2_lbl, "Analog2-2", bool(state["analog2_2_seen"]))

        self._vals["pos1"].setText(f"{state['pos1_rad']:.4f}")
        self._vals["pos2"].setText(f"{state['pos2_rad']:.4f}")
        self._vals["vel1"].setText(f"{state['vel1_rads']:.4f}")
        self._vals["vel2"].setText(f"{state['vel2_rads']:.4f}")
        self._vals["trq1"].setText(f"{state['torque1_Nm']:.4f}")
        self._vals["trq2"].setText(f"{state['torque2_Nm']:.4f}")
        self._vals["a1"].setText(f"{state['analog2_1']:.0f}")
        self._vals["a2"].setText(f"{state['analog2_2']:.0f}")
        self._vals["cmd"].setText(f"{state['cmd_raw_mNm']:.0f}")
        self._vals["time"].setText(f"{state['global_time_s']:.4f}")
        self._vals["dt"].setText(f"{state['global_dt_s']:.6f}")

        for i, value in enumerate(state["emg"]):
            self._emg_vals[i].setText(f"{value:.4f}")

    def _set_indicator(self, label, name, ok):
        color = "#1ec97f" if ok else "#d65265"
        label.setText(f"{name} {'●' if ok else '○'}")
        label.setStyleSheet(
            f"border:1px solid {color}; color:{color}; border-radius:5px; "
            "padding:6px 10px; font-size:11px;"
        )

    def _exit(self):
        self._writer.send(state=STATE_IDLE, folder=self._folder, cmd_raw=0)
        self._timer.stop()
        self._reader.close()
        self._writer.close()
        QApplication.quit()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    dlg = SessionDialog()
    if dlg.exec() != QDialog.DialogCode.Accepted:
        sys.exit(0)

    try:
        reader = ShmReader()
        writer = ShmWriter()
    except Exception as exc:
        print(exc)
        sys.exit("Make sure demo_main.exe is running before display.py.")

    window = MainWindow(reader, writer, dlg.folder, dlg.command_raw)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
