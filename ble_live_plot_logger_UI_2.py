# -*- coding: utf-8 -*-
"""
PyQt5 + PyQtGraph 기반 4채널 실시간 플로터
- USB-UART 통해 보드 데이터 수신 (CSV: "v1,v2,v3,v4\n")
- Start: 'on\n' 전송 + CSV 로깅 시작
- Stop : 'off\n' 전송 + CSV 로깅 종료
- Stim : "pulses,period,duty\n" 전송
- CH1~CH4 가시성 토글, 마지막 CSV 열기 지원
- Merge / Split 버튼 추가 (그래프 합치기/분리)
- 채널별 색상 고정, 스크롤/줌 가능
"""

import sys
import os
import csv
import threading
from collections import deque
from datetime import datetime

import serial
from serial import SerialException

import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

# ===== 설정 =====
PORT = "COM3"              # 시리얼 포트
BAUDRATE = 115200
MAX_POINTS = 500           # 화면에 보일 포인트 개수
UPDATE_INTERVAL_MS = 100   # 플롯 업데이트 주기
Y_MIN, Y_MAX = 0, 3000     # Y축 범위
CHANNEL_COLORS = ['r', 'g', 'b', 'y']  # CH1~CH4 색상


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live ADC Data (4CH) - PyQt5 + PyQtGraph")

        # 상태 변수
        self.data_queues = [deque([0]*MAX_POINTS, maxlen=MAX_POINTS) for _ in range(4)]
        self.channel_visible = [True, True, True, True]
        self.logging_enabled = False
        self.streaming_enabled = False
        self.csv_file = None
        self.csv_writer = None
        self.last_csv_filename = None
        self.ser = None
        self.reader_thread = None
        self.reader_stop = threading.Event()
        self.merged = False  # Merge 상태

        # ========== UI 구성 ==========
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)

        # ---- 상단 컨트롤 영역 ----
        ctrl = QtWidgets.QHBoxLayout()

        # Stim 파라미터
        ctrl.addWidget(QtWidgets.QLabel("Pulse"))
        self.pulse_edit = QtWidgets.QLineEdit()
        self.pulse_edit.setFixedWidth(70)
        ctrl.addWidget(self.pulse_edit)

        ctrl.addWidget(QtWidgets.QLabel("Period"))
        self.period_edit = QtWidgets.QLineEdit()
        self.period_edit.setFixedWidth(70)
        ctrl.addWidget(self.period_edit)

        ctrl.addWidget(QtWidgets.QLabel("Duty"))
        self.duty_edit = QtWidgets.QLineEdit()
        self.duty_edit.setFixedWidth(70)
        ctrl.addWidget(self.duty_edit)

        self.btn_stim = QtWidgets.QPushButton("Stim")
        self.btn_stim.clicked.connect(self.send_stimulus)
        ctrl.addWidget(self.btn_stim)

        ctrl.addStretch()

        # Start / Stop 버튼
        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_start.clicked.connect(self.start_logging)
        ctrl.addWidget(self.btn_start)

        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_stop.clicked.connect(self.stop_logging)
        ctrl.addWidget(self.btn_stop)

        # Merge / Split 버튼
        self.btn_merge_split = QtWidgets.QPushButton("Merge")
        self.btn_merge_split.clicked.connect(self.toggle_merge_split)
        ctrl.addWidget(self.btn_merge_split)

        # 채널 체크박스
        self.cb_ch = []
        for i in range(4):
            cb = QtWidgets.QCheckBox(f"CH{i+1}")
            cb.setChecked(True)
            cb.stateChanged.connect(lambda state, idx=i: self.toggle_channel(idx, state))
            self.cb_ch.append(cb)
            ctrl.addWidget(cb)

        # Open CSV 버튼
        self.btn_open = QtWidgets.QPushButton("Open CSV")
        self.btn_open.clicked.connect(self.open_csv)
        ctrl.addWidget(self.btn_open)

        vbox.addLayout(ctrl)

        # ---- 그래프 영역 ----
        self.graphics = pg.GraphicsLayoutWidget()
        vbox.addWidget(self.graphics)

        # 기본 Split 상태에서 그래프 생성
        self.create_split_plots()

        # ---- 상태바 ----
        self.status = self.statusBar()
        self.status.showMessage("Ready")

        # ---- 타이머 (그래프 업데이트) ----
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(UPDATE_INTERVAL_MS)

        # ---- 시리얼 연결 시도 ----
        self.try_open_serial()

        # ---- 리더 스레드 시작 ----
        self.start_reader_thread()

        # 종료 시 정리
        self.destroyed.connect(self.cleanup)
        self.resize(1000, 800)

    # ===== 시리얼 =====
    def try_open_serial(self):
        try:
            self.ser = serial.Serial(PORT, BAUDRATE, timeout=1)
            self.status.showMessage(f"Serial opened: {PORT} @ {BAUDRATE}")
        except SerialException as e:
            self.ser = None
            self.status.showMessage(f"Serial open failed: {e}")

    def start_reader_thread(self):
        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

    def reader_loop(self):
        """시리얼에서 데이터 읽어오기"""
        while not self.reader_stop.is_set():
            if self.ser is None:
                QtCore.QThread.msleep(200)
                continue
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line or "," not in line:
                    continue
                parts = [p.strip() for p in line.split(",")]
                if len(parts) != 4:
                    continue
                try:
                    values = [int(x) for x in parts]
                except ValueError:
                    continue

                if self.streaming_enabled:
                    for i in range(4):
                        self.data_queues[i].append(values[i])

                    if self.logging_enabled and self.csv_writer:
                        self.csv_writer.writerow(values)
            except Exception:
                pass

    # ===== 버튼 핸들러 =====
    def start_logging(self):
        if self.ser is None:
            self.try_open_serial()
            if self.ser is None:
                QtWidgets.QMessageBox.warning(self, "Serial", "Serial open failed.")
                return

        if not self.logging_enabled:
            filename = f"adc_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.last_csv_filename = filename
            try:
                self.csv_file = open(filename, mode="w", newline="")
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(["CH1", "CH2", "CH3", "CH4"])
            except Exception as e:
                QtWidgets.QMessageBox.warning(self, "CSV", f"Failed to open CSV: {e}")
                return

            try:
                self.ser.write(b"on\n")
            except Exception:
                pass

            self.logging_enabled = True
            self.streaming_enabled = True
            self.status.showMessage(f"🟢 Logging started → {filename}")

    def stop_logging(self):
        if self.logging_enabled:
            try:
                if self.ser:
                    self.ser.write(b"off\n")
            except Exception:
                pass

            self.logging_enabled = False
            self.streaming_enabled = False
            try:
                if self.csv_file:
                    self.csv_file.close()
            finally:
                self.csv_file = None
                self.csv_writer = None
            self.status.showMessage("🛑 Logging stopped. CSV file closed.")

    def send_stimulus(self):
        pulses = self.pulse_edit.text().strip()
        period = self.period_edit.text().strip()
        duty = self.duty_edit.text().strip()

        if not (pulses.isdigit() and period.isdigit() and duty.isdigit()):
            QtWidgets.QMessageBox.warning(self, "Stim", "Invalid input. Use integers.")
            return

        cmd = f"{pulses},{period},{duty}\n".encode()
        try:
            if self.ser is None:
                self.try_open_serial()
            if self.ser:
                self.ser.write(cmd)
                self.status.showMessage(f"📤 Sent stimulus: {pulses},{period},{duty}")
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Serial", f"Write failed: {e}")

    def toggle_channel(self, idx, state):
        self.channel_visible[idx] = (state == QtCore.Qt.Checked)

    def open_csv(self):
        if self.last_csv_filename and os.path.exists(self.last_csv_filename):
            try:
                os.startfile(self.last_csv_filename)  # Windows
            except AttributeError:
                import subprocess
                opener = "open" if sys.platform == "darwin" else "xdg-open"
                subprocess.call([opener, self.last_csv_filename])
        else:
            QtWidgets.QMessageBox.information(self, "CSV", "No CSV file to open.")

    # ===== 그래프 생성 =====
    def create_split_plots(self):
        """4채널 개별 plot 생성 (색상 유지, 스크롤/줌 가능)"""
        self.graphics.clear()
        self.plots = []
        self.curves = []
        for i in range(4):
            p = self.graphics.addPlot(row=i, col=0)
            p.showGrid(x=True, y=True)
            p.setLabel("left", f"CH{i+1}")
            if i == 3:
                p.setLabel("bottom", "Samples")
            p.setYRange(Y_MIN, Y_MAX)
            # ViewBox 스크롤/줌 활성화
            p.setMouseEnabled(x=True, y=True)
            curve = p.plot([], [], pen=CHANNEL_COLORS[i])
            self.plots.append(p)
            self.curves.append(curve)

    def toggle_merge_split(self):
        """그래프 합치기 / 분리"""
        self.graphics.clear()
        if not self.merged:
            # Merge: 모든 채널 하나 그래프
            p = self.graphics.addPlot(row=0, col=0)
            p.showGrid(x=True, y=True)
            p.setLabel("left", "ADC Value")
            p.setLabel("bottom", "Samples")
            p.setYRange(Y_MIN, Y_MAX)
            p.setMouseEnabled(x=True, y=True)
            self.curves = []
            for i in range(4):
                curve = p.plot([], [], pen=CHANNEL_COLORS[i], name=f"CH{i+1}")
                self.curves.append(curve)
            self.merged = True
            self.btn_merge_split.setText("Split")
        else:
            # Split: 각 채널별 plot 재생성
            self.create_split_plots()
            self.merged = False
            self.btn_merge_split.setText("Merge")

    # ===== 플롯 업데이트 =====
    def update_plot(self):
        for i in range(len(self.curves)):
            if self.channel_visible[i]:
                y = list(self.data_queues[i])
                x = list(range(len(y)))
                self.curves[i].setData(x, y, connect="finite")
            else:
                self.curves[i].setData([], [])

    # ===== 종료 =====
    def cleanup(self):
        self.reader_stop.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass


def main():
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, useOpenGL=False)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
