"""
레이저 변위센서 곡률 측정 시스템 - PC GUI

STM32에서 UART로 받은 스캔 데이터를 단면 형상 그래프로 그리고,
평탄도(Bow) / 곡률(Curvature) 결과와 이상치(임계값 0.05mm 초과)를 표시한다.

UART 프로토콜 (ASCII, \r\n 종단)
  PC  -> MCU : 'M'                                  스캔 시작
  MCU -> PC  : RTD,<거리 um>                         대기 중 실시간 거리 (100ms)
  MCU -> PC  : SCAN,<서보 펄스 us>,<거리 um>          스캔 데이터 (30ms)
  MCU -> PC  : RES,<P1>,<P2>,<P3>,<Bow um>,<곡률 x1e6> 연산 결과
"""

import math
import sys
from collections import deque

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QHBoxLayout, QLabel, QPushButton,
    QVBoxLayout, QWidget,
)

# 서보 펄스 -> 각도 변환 (펌웨어와 동일한 값)
PULSE_CENTER = 1475
PULSE_PER_30DEG = 333

# 불량 판정 임계값 (mm)
THRESH = 0.05
# 이동평균 창 크기 (이상치 검출용)
MA_WINDOW = 9

RTD_HISTORY = 300


class SerialReader(QThread):
    """시리얼 포트에서 한 줄씩 읽어 GUI로 전달하는 스레드"""

    line_received = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True

    def run(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                self.error.emit(str(e))
                break
            if raw:
                line = raw.decode("ascii", errors="ignore").strip()
                if line:
                    self.line_received.emit(line)

    def stop(self):
        self.running = False
        self.wait(500)


class ScannerWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("레이저 변위센서 곡률 측정 시스템")
        self.resize(1280, 800)

        self.ser = None
        self.reader = None

        self.profile_x = []
        self.profile_y = []
        self.rtd_data = deque(maxlen=RTD_HISTORY)

        pg.setConfigOptions(antialias=True)
        self._build_ui()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        layout = QHBoxLayout(self)

        # 왼쪽 제어 패널
        ctrl_panel = QVBoxLayout()
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200"])

        self.btn_connect = QPushButton("연결")
        self.btn_connect.clicked.connect(self.toggle_connection)

        self.btn_scan = QPushButton("스캔 시작")
        self.btn_scan.clicked.connect(self.start_scan)
        self.btn_scan.setEnabled(False)

        self.btn_clear = QPushButton("그래프 초기화")
        self.btn_clear.clicked.connect(self.clear_graphs)

        self.lbl_status = QLabel("상태: 대기중")
        self.lbl_status.setStyleSheet("color: gray; font-weight: bold;")

        lbl_style = "font-size: 14pt; font-weight: bold; margin-top: 10px;"
        self.lbl_p1 = QLabel("P1 : -")
        self.lbl_p2 = QLabel("P2 : -")
        self.lbl_p3 = QLabel("P3 : -")
        self.lbl_bow = QLabel("Bow : -")
        self.lbl_curv = QLabel("Curv : -")
        self.lbl_judge = QLabel("판정 : -")
        for lbl in (self.lbl_p1, self.lbl_p2, self.lbl_p3,
                    self.lbl_bow, self.lbl_curv, self.lbl_judge):
            lbl.setStyleSheet(lbl_style)
        self.lbl_bow.setStyleSheet(lbl_style + "color: blue;")

        ctrl_panel.addWidget(QLabel("COM 포트"))
        ctrl_panel.addWidget(self.port_combo)
        ctrl_panel.addWidget(QLabel("Baudrate"))
        ctrl_panel.addWidget(self.baud_combo)
        ctrl_panel.addWidget(self.btn_connect)
        ctrl_panel.addSpacing(10)
        ctrl_panel.addWidget(self.btn_scan)
        ctrl_panel.addWidget(self.btn_clear)
        for lbl in (self.lbl_p1, self.lbl_p2, self.lbl_p3,
                    self.lbl_bow, self.lbl_curv, self.lbl_judge):
            ctrl_panel.addWidget(lbl)
        ctrl_panel.addStretch(1)
        ctrl_panel.addWidget(self.lbl_status)
        layout.addLayout(ctrl_panel, 1)

        # 오른쪽 그래프 패널
        graph_panel = QVBoxLayout()

        self.plot_profile = pg.PlotWidget(title="물체 단면 형상 (Profile)")
        self.plot_profile.setLabel("bottom", "Width (mm)")
        self.plot_profile.setLabel("left", "Height (mm)")
        self.plot_profile.showGrid(x=True, y=True)
        self.plot_profile.setAspectLocked(True)
        self.plot_profile.invertY(True)
        self.curve_profile = self.plot_profile.plot(
            pen=None, symbol="o", symbolSize=5, symbolBrush="b"
        )
        # 임계값 초과 지점(빨간 점)
        self.curve_profile_bend = self.plot_profile.plot(
            pen=None, symbol="o", symbolSize=9, symbolBrush="r"
        )

        self.plot_rtd = pg.PlotWidget(title="실시간 거리 센서 값")
        self.plot_rtd.setLabel("bottom", "Time")
        self.plot_rtd.setLabel("left", "Distance (mm)")
        self.plot_rtd.enableAutoRange()
        self.plot_rtd.invertY(True)
        self.curve_rtd = self.plot_rtd.plot(pen="r")

        graph_panel.addWidget(self.plot_profile, 2)
        graph_panel.addWidget(self.plot_rtd, 1)
        layout.addLayout(graph_panel, 4)

        self.update_ports()

    def update_ports(self):
        self.port_combo.clear()
        for port in serial.tools.list_ports.comports():
            self.port_combo.addItem(port.device)

    def set_status(self, text, color="gray"):
        self.lbl_status.setText(f"상태: {text}")
        self.lbl_status.setStyleSheet(f"color: {color}; font-weight: bold;")

    # ------------------------------------------------------------ Serial
    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect_serial()
            return

        port = self.port_combo.currentText()
        if not port:
            self.update_ports()
            self.set_status("포트 없음", "red")
            return
        try:
            self.ser = serial.Serial(port, int(self.baud_combo.currentText()), timeout=0.1)
        except serial.SerialException as e:
            self.set_status(f"연결 실패 ({e})", "red")
            return

        self.reader = SerialReader(self.ser)
        self.reader.line_received.connect(self.handle_line)
        self.reader.error.connect(self.handle_serial_error)
        self.reader.start()

        self.btn_connect.setText("연결 해제")
        self.btn_scan.setEnabled(True)
        self.set_status(f"{port} 연결됨", "green")

    def disconnect_serial(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.ser:
            self.ser.close()
            self.ser = None
        self.btn_connect.setText("연결")
        self.btn_scan.setEnabled(False)
        self.set_status("대기중")

    def handle_serial_error(self, msg):
        self.disconnect_serial()
        self.set_status(f"통신 오류 ({msg})", "red")

    def start_scan(self):
        if not (self.ser and self.ser.is_open):
            return
        self.clear_profile()
        self.ser.write(b"M")
        self.btn_scan.setEnabled(False)
        self.set_status("스캔 중...", "orange")

    # ------------------------------------------------------- Data handling
    def handle_line(self, line):
        parts = line.split(",")
        try:
            if parts[0] == "RTD" and len(parts) == 2:
                self.on_rtd(int(parts[1]) / 1000.0)
            elif parts[0] == "SCAN" and len(parts) == 3:
                self.on_scan(int(parts[1]), int(parts[2]) / 1000.0)
            elif parts[0] == "RES" and len(parts) == 6:
                self.on_result([int(v) for v in parts[1:]])
        except ValueError:
            pass  # 깨진 라인 무시

    def on_rtd(self, dist_mm):
        self.rtd_data.append(dist_mm)
        self.curve_rtd.setData(list(self.rtd_data))

    def on_scan(self, pulse, dist_mm):
        # 서보 펄스 -> 각도 -> (x, y) 좌표
        angle_deg = (pulse - PULSE_CENTER) * 30.0 / PULSE_PER_30DEG
        rad = math.radians(angle_deg)
        self.profile_x.append(dist_mm * math.sin(rad))
        self.profile_y.append(dist_mm * math.cos(rad))
        self.curve_profile.setData(self.profile_x, self.profile_y)
        self.on_rtd(dist_mm)

    def on_result(self, values):
        p1, p2, p3, bow_um, curv_e6 = values
        bow_mm = bow_um / 1000.0
        curv = curv_e6 / 1e6  # 1/mm

        self.lbl_p1.setText(f"P1 : {p1 / 1000.0:.3f} mm")
        self.lbl_p2.setText(f"P2 : {p2 / 1000.0:.3f} mm")
        self.lbl_p3.setText(f"P3 : {p3 / 1000.0:.3f} mm")
        self.lbl_bow.setText(f"Bow : {bow_mm:.3f} mm")
        if curv != 0:
            self.lbl_curv.setText(f"Curv : R = {abs(1.0 / curv):.1f} mm")
        else:
            self.lbl_curv.setText("Curv : R = ∞ (평면)")

        outliers = self.highlight_outliers()
        if outliers > 0:
            self.lbl_judge.setText(f"판정 : NG ({outliers}점)")
            self.lbl_judge.setStyleSheet("font-size: 14pt; font-weight: bold; color: red;")
        else:
            self.lbl_judge.setText("판정 : OK")
            self.lbl_judge.setStyleSheet("font-size: 14pt; font-weight: bold; color: green;")

        self.btn_scan.setEnabled(True)
        self.set_status("스캔 완료", "green")

    def highlight_outliers(self):
        """이동평균 대비 잔차가 임계값(0.05mm)을 넘는 지점을 빨간 점으로 표시"""
        if len(self.profile_y) < MA_WINDOW:
            return 0

        x_data = np.array(self.profile_x)
        y_data = np.array(self.profile_y)

        kernel = np.ones(MA_WINDOW) / MA_WINDOW
        pad = MA_WINDOW // 2
        y_padded = np.pad(y_data, pad, mode="edge")
        y_ma = np.convolve(y_padded, kernel, mode="valid")
        residuals = np.abs(y_data - y_ma)

        mask = residuals > THRESH

        bad_x = x_data[mask]
        bad_y = y_data[mask]
        self.curve_profile_bend.setData(bad_x, bad_y)
        print(f"[MA Highlight] Found {len(bad_x)} outlier points.")
        return int(mask.sum())

    def clear_profile(self):
        self.profile_x.clear()
        self.profile_y.clear()
        self.curve_profile.setData([], [])
        self.curve_profile_bend.setData([], [])

    def clear_graphs(self):
        self.clear_profile()
        self.rtd_data.clear()
        self.curve_rtd.setData([])
        for lbl, name in ((self.lbl_p1, "P1"), (self.lbl_p2, "P2"), (self.lbl_p3, "P3"),
                          (self.lbl_bow, "Bow"), (self.lbl_curv, "Curv"), (self.lbl_judge, "판정")):
            lbl.setText(f"{name} : -")

    def closeEvent(self, event):
        self.disconnect_serial()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ScannerWindow()
    win.show()
    sys.exit(app.exec_())
