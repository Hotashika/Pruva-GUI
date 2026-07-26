import os
import time
import html
import math
from datetime import datetime
from pathlib import Path

from PyQt5 import uic
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QBrush, QFont, QPainter, QPen, QPixmap
from PyQt5.QtGui import QPolygonF
from PyQt5.QtWidgets import QDialog, QFileDialog, QMainWindow, QTableWidgetItem

from ..core.data_system import NjordVeriSistemi


PAKET_KLASORU = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJE_KLASORU = os.path.dirname(PAKET_KLASORU)
KAYNAK_KLASORU = os.path.join(PROJE_KLASORU, "assets")
UI_KLASOR = os.path.join(KAYNAK_KLASORU, "ui")
GORSEL_KLASORU = os.path.join(KAYNAK_KLASORU, "images")
WAYPOINT_KLASORU = os.path.join(PROJE_KLASORU, "waypoints", "waypoints")

ALIGN_CENTER = getattr(getattr(Qt, "AlignmentFlag", None), "AlignCenter", None)
if ALIGN_CENTER is None:
    ALIGN_CENTER = getattr(Qt, "AlignCenter", 0)

KEEP_ASPECT = getattr(getattr(Qt, "AspectRatioMode", None), "KeepAspectRatio", None)
if KEEP_ASPECT is None:
    KEEP_ASPECT = getattr(Qt, "KeepAspectRatio", 1)

SMOOTH = getattr(getattr(Qt, "TransformationMode", None), "SmoothTransformation", None)
if SMOOTH is None:
    SMOOTH = getattr(Qt, "SmoothTransformation", 1)

MAP_IMAGE_FILE = "ecdat_map.png"
MAP_IMAGE_SIZE = (592, 832)
BASE_UI_SIZE = (1680, 945)
MAP_CALIBRATION_POINTS = [
    {"name": "WP1", "lat": 37.9524548, "lon": 32.5009435, "pixel": (260, 92)},
    {"name": "WP2", "lat": 37.9524210, "lon": 32.5015175, "pixel": (475, 98)},
    {"name": "WP3", "lat": 37.9514904, "lon": 32.5013566, "pixel": (421, 482)},
    {"name": "WP4", "lat": 37.9510082, "lon": 32.5012600, "pixel": (376, 704)},
    {"name": "WP5", "lat": 37.9510589, "lon": 32.5006807, "pixel": (190, 694)},
    {"name": "WP6", "lat": 37.9516681, "lon": 32.5006807, "pixel": (194, 374)},
]
# 6S 10 Ah battery: 22.2 V nominal x 10 Ah = 222 Wh.
DEFAULT_BATTERY_CAPACITY_WH = 222.0
BATTERY_EMPTY_VOLTAGE = 21.0
BATTERY_FULL_VOLTAGE = 25.2
BATTERY_SOC_CURVE = (
    (21.0, 0),
    (21.6, 10),
    (22.2, 20),
    (22.8, 40),
    (23.4, 60),
    (24.0, 80),
    (24.6, 90),
    (25.2, 100),
)


def _pil_yuzdesi_voltajdan(voltage):
    voltage = float(voltage or 0.0)
    if voltage <= BATTERY_SOC_CURVE[0][0]:
        return 0
    if voltage >= BATTERY_SOC_CURVE[-1][0]:
        return 100
    for (low_v, low_percent), (high_v, high_percent) in zip(
        BATTERY_SOC_CURVE,
        BATTERY_SOC_CURVE[1:],
    ):
        if low_v <= voltage <= high_v:
            ratio = (voltage - low_v) / (high_v - low_v)
            percent = low_percent + ratio * (high_percent - low_percent)
            return max(0, min(int(round(percent)), 100))
    return 0


def _patch_pyqt5_uic_enums():
    # Some .ui files were saved with enum names that PyQt5's uic does not know.
    # Adding the aliases here keeps direct imports and main_njord.py launches consistent.
    aliases = {
        "Dec": 1,
        "QLCDNumber::Mode::Dec": QtWidgets.QLCDNumber.Dec,
        "Flat": 0,
        "QDialogButtonBox::StandardButton::Cancel": QtWidgets.QDialogButtonBox.Cancel,
        "QDialogButtonBox::StandardButton::Ok": QtWidgets.QDialogButtonBox.Ok,
    }
    for name, value in aliases.items():
        if not hasattr(QtCore.Qt, name):
            setattr(QtCore.Qt, name, value)


_patch_pyqt5_uic_enums()


class PortEkrani(QDialog):
    def __init__(self, veri_sistemi):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, "port.ui"), self)
        self.veri_sistemi = veri_sistemi
        self.setWindowTitle("CONNECTIVITY")
        self.setWindowFlag(Qt.WindowContextHelpButtonHint, False)
        if hasattr(self, "pushButton_2"):
            self.pushButton_2.hide()
        if hasattr(self, "pushButton"):
            self.pushButton.hide()
        self._varsayilan_portlari_ayarla()
        self._baglanti_durumuna_gore_duzenle()
        if hasattr(self, "pushButton_2"):
            self.pushButton_2.clicked.connect(self.iptal)
        self.buttonBox.accepted.connect(self.onayla)
        self.buttonBox.rejected.connect(self.iptal)

    def _varsayilan_portlari_ayarla(self):
        if self.comboBox.findText("COM") >= 0:
            self.comboBox.setCurrentText("COM")
        if self.comboBox_2.findText("57600") >= 0:
            self.comboBox_2.setCurrentText("57600")

        portlar = self._seri_portlari_bul()
        if portlar:
            secili = portlar[0]
            self.lineEdit.setText(secili["device"])
            adaylar = ", ".join(
                f"{port['device']} ({port['description']})" for port in portlar[:4]
            )
            self.veri_sistemi.log_sinyali.emit(f"SERIAL PORTS FOUND: {adaylar}")
            if secili["is_rfd"]:
                self.veri_sistemi.log_sinyali.emit(
                    f"RFD RADIO AUTO-DETECTED: {secili['device']} ({secili['description']})"
                )
            else:
                self.veri_sistemi.log_sinyali.emit(
                    f"INFO: Using best detected serial port: {secili['device']} ({secili['description']})"
                )
        elif not self.lineEdit.text().strip():
            self.lineEdit.setText("COM6")
            self.veri_sistemi.log_sinyali.emit("WARNING: No serial port was detected.")


    @staticmethod
    def _seri_port_puani(port):
        alanlar = " ".join(
            str(getattr(port, alan, "") or "")
            for alan in ("description", "manufacturer", "product", "interface", "hwid")
        ).lower()
        if any(ipucu in alanlar for ipucu in ("rfd900", "rfd868", "rfd radio", "rfdesign", "rf design")):
            return 100
        if any(ipucu in alanlar for ipucu in ("ardupilot", "pixhawk", "fmuv", "holybro")):
            return 95
        if "sik" in alanlar and any(ipucu in alanlar for ipucu in ("radio", "telemetry", "modem")):
            return 80
        if "telemetry radio" in alanlar:
            return 70
        if any(
            ipucu in alanlar
            for ipucu in (
                "cp210",
                "silicon labs",
                "ftdi",
                "usb serial",
                "usb-serial",
                "usb seri",
                "usb-seri",
                "ch340",
                "wch",
            )
        ):
            return 60
        device = str(getattr(port, "device", "") or "").lower()
        if "bluetooth" in alanlar:
            return -10
        if "ttyusb" in device or "ttyacm" in device:
            return 10
        if device.startswith("com"):
            return 5
        return 0

    def _seri_portlari_bul(self):
        try:
            from serial.tools import list_ports
            sonuclar = []
            for sira, port in enumerate(list_ports.comports()):
                puan = self._seri_port_puani(port)
                sonuclar.append(
                    {
                        "device": port.device,
                        "description": str(getattr(port, "description", "") or "Unknown"),
                        "is_rfd": puan >= 70,
                        "score": puan,
                        "order": sira,
                    }
                )
            return sorted(sonuclar, key=lambda item: (-item["score"], item["order"]))
        except Exception as exc:
            self.veri_sistemi.log_sinyali.emit(f"WARNING: Serial port scan failed: {exc}")
            return []

    def onayla(self):
        if self.veri_sistemi.durum_al().get("baglanti"):
            self.veri_sistemi.baglanti_kes()
            self.accept()
            return

        self.veri_sistemi.baglanti_kur(
            self.comboBox.currentText(),
            self.comboBox_2.currentText(),
            self.lineEdit.text(),
        )
        self.accept()

    def iptal(self):
        self.reject()

    def _baglanti_durumuna_gore_duzenle(self):
        ok_button = self.buttonBox.button(QtWidgets.QDialogButtonBox.Ok)
        cancel_button = self.buttonBox.button(QtWidgets.QDialogButtonBox.Cancel)
        if cancel_button is not None:
            cancel_button.setText("Cancel")
        if ok_button is None:
            return

        if self.veri_sistemi.durum_al().get("baglanti"):
            ok_button.setText("DISCONNECT")
        else:
            ok_button.setText("CONNECT")


class AlgoritmaTespitGrafigi(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._detections = []
        self.setMinimumHeight(130)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def set_detections(self, detections):
        self._detections = [item for item in (detections or []) if isinstance(item, dict)][-8:]
        self.update()

    @staticmethod
    def _class_color(class_name):
        name = str(class_name or "").lower()
        if "red" in name:
            return QColor(231, 76, 60)
        if "green" in name:
            return QColor(46, 204, 113)
        if "orange" in name:
            return QColor(243, 126, 32)
        if "yellow" in name:
            return QColor(241, 196, 15)
        if "black" in name:
            return QColor(120, 130, 140)
        return QColor(0, 210, 255)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(8, 27, 38))

        center = QPointF(self.width() / 2.0, self.height() - 24.0)
        radius = max(30.0, min(self.width() * 0.46, self.height() - 45.0))
        painter.setPen(QPen(QColor(88, 155, 180, 100), 1))
        for fraction in (0.25, 0.5, 0.75, 1.0):
            r = radius * fraction
            painter.drawArc(QtCore.QRectF(center.x() - r, center.y() - r, 2 * r, 2 * r), 0, 180 * 16)
        for angle in (-60, -30, 0, 30, 60):
            radians = math.radians(angle)
            end = QPointF(center.x() + math.sin(radians) * radius, center.y() - math.cos(radians) * radius)
            painter.drawLine(center, end)

        painter.setPen(QPen(QColor(0, 210, 255), 2))
        painter.setBrush(QBrush(QColor(0, 120, 180)))
        vessel = QPolygonF([QPointF(center.x(), center.y() - 14), QPointF(center.x() + 9, center.y() + 7), QPointF(center.x() - 9, center.y() + 7)])
        painter.drawPolygon(vessel)

        if not self._detections:
            painter.setPen(QPen(QColor(220, 230, 235), 1))
            painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
            painter.drawText(self.rect(), Qt.AlignCenter, "WAITING FOR DETECTIONS")
            return

        valid_depths = []
        for item in self._detections:
            try:
                depth = float(item.get("depth", 0.0))
                if math.isfinite(depth) and depth >= 0.0:
                    valid_depths.append(depth)
            except (TypeError, ValueError):
                continue
        max_depth = max(10.0, max(valid_depths, default=10.0))

        legend_items = []
        for index, item in enumerate(self._detections, start=1):
            try:
                angle = float(item.get("angle", 0.0))
                depth = float(item.get("depth", 0.0))
                confidence = max(0.0, min(float(item.get("confidence", 0.0)), 1.0))
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (angle, depth, confidence)) or depth < 0.0:
                continue
            r = min(depth / max_depth, 1.0) * radius
            radians = math.radians(max(-90.0, min(angle, 90.0)))
            point = QPointF(center.x() + math.sin(radians) * r, center.y() - math.cos(radians) * r)
            name = str(item.get("class_name") or "OBJECT")[:18]
            color = self._class_color(name)
            painter.setPen(QPen(QColor(255, 255, 255), 1))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(point, 8, 8)
            painter.setPen(QPen(QColor(255, 255, 255), 1))
            painter.setFont(QFont("Segoe UI", 7, QFont.Bold))
            painter.drawText(
                QtCore.QRectF(point.x() - 8, point.y() - 8, 16, 16),
                Qt.AlignCenter,
                str(index),
            )
            legend_items.append(
                (color, f"{index}  {name} | {depth:.1f} m | {angle:+.1f}° | {confidence * 100:.0f}%")
            )

        latest = self._detections[-1]
        painter.setPen(QPen(QColor(230, 240, 245), 1))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(
            10,
            18,
            f"CURRENT FRAME: {len(self._detections)} DETECTION(S) | ID: {latest.get('frame_id', '--')}",
        )

        if legend_items:
            legend_height = 8 + len(legend_items) * 17
            legend_width = min(max(250, self.width() - 20), 390)
            painter.fillRect(QtCore.QRectF(8, 25, legend_width, legend_height), QColor(8, 27, 38, 220))
            painter.setFont(QFont("Segoe UI", 7, QFont.Bold))
            for row, (color, label) in enumerate(legend_items):
                y = 40 + row * 17
                painter.setPen(QPen(color, 3))
                painter.drawLine(14, y - 4, 24, y - 4)
                painter.setPen(QPen(QColor(235, 242, 245), 1))
                painter.drawText(30, y, label)


class HaritaCizimKatmani(QtWidgets.QWidget):
    TRAIL_MIN_DELTA = 0.000005
    COG_MIN_SPEED_M_S = 0.30

    def __init__(self, parent=None, map_pixmap=None):
        super().__init__(parent)
        self._vehicle = None
        self._waypoints = []
        self._trail = []
        self._map_pixmap = map_pixmap if map_pixmap is not None else QPixmap()
        self._affine_x, self._affine_y = self._kalibrasyon_hazirla()
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WA_TranslucentBackground, True)

    def set_vehicle(self, vehicle):
        self._vehicle = vehicle
        if self._valid_coordinate(vehicle):
            son = self._trail[-1] if self._trail else None
            if not son or abs(float(son["lat"]) - float(vehicle["lat"])) > self.TRAIL_MIN_DELTA or abs(float(son["lon"]) - float(vehicle["lon"])) > self.TRAIL_MIN_DELTA:
                self._trail.append({"lat": float(vehicle["lat"]), "lon": float(vehicle["lon"])})
                # Keep enough history to show the complete competition run,
                # rather than only the last short segment of the vessel path.
                self._trail = self._trail[-2000:]
        self.update()

    def set_waypoints(self, waypoints):
        self._waypoints = waypoints or []
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        self._harita_arka_plani_ciz(painter)
        self._arka_plan_efekti_ciz(painter)

        points = self._valid_points()
        if not points:
            self._bos_durum_ciz(painter)
            return

        waypoint_pixels = []
        for wp in self._waypoints:
            if self._valid_coordinate(wp):
                waypoint_pixels.append((wp, self._to_pixel(wp["lat"], wp["lon"])))

        if len(waypoint_pixels) > 1:
            self._rota_ciz(painter, [p for _, p in waypoint_pixels])

        trail_pixels = [self._to_pixel(p["lat"], p["lon"]) for p in self._trail]
        if len(trail_pixels) > 1:
            self._trail_ciz(painter, trail_pixels)
        self._trail_durumu_ciz(painter)
        self._aciklama_ciz(painter)

        for index, (wp, point) in enumerate(waypoint_pixels, start=1):
            self._waypoint_ciz(painter, point, wp.get("name") or f"WP{index}", index)

        if self._vehicle and self._valid_coordinate(self._vehicle):
            point = self._to_pixel(self._vehicle["lat"], self._vehicle["lon"])
            self._arac_ciz(
                painter,
                point,
                self._vehicle.get("yaw", 0.0),
                self._vehicle.get("cog"),
                self._vehicle.get("speed", 0.0),
            )

    def _valid_points(self):
        points = [wp for wp in self._waypoints if self._valid_coordinate(wp)]
        if self._vehicle and self._valid_coordinate(self._vehicle):
            points.append(self._vehicle)
        return points

    def _valid_coordinate(self, point):
        try:
            lat = float(point.get("lat"))
            lon = float(point.get("lon"))
        except (TypeError, ValueError, AttributeError):
            return False
        return abs(lat) > 0.000001 or abs(lon) > 0.000001

    def _to_pixel(self, lat, lon):
        native_x, native_y = self._to_native(lat, lon)
        view = self._gorunum_rect()
        scale_x = max(self.width(), 1) / max(view.width(), 1.0)
        scale_y = max(self.height(), 1) / max(view.height(), 1.0)
        return QPointF((native_x - view.left()) * scale_x, (native_y - view.top()) * scale_y)

    def _to_native(self, lat, lon):
        native_x = self._affine_x[0] * float(lat) + self._affine_x[1] * float(lon) + self._affine_x[2]
        native_y = self._affine_y[0] * float(lat) + self._affine_y[1] * float(lon) + self._affine_y[2]
        return native_x, native_y

    def _gorunum_rect(self):
        native_points = [self._to_native(p["lat"], p["lon"]) for p in self._valid_points()]
        native_points.extend(self._to_native(p["lat"], p["lon"]) for p in self._trail)
        if not native_points:
            return QtCore.QRectF(0.0, 0.0, *MAP_IMAGE_SIZE)

        xs = [p[0] for p in native_points]
        ys = [p[1] for p in native_points]
        min_width = 220.0
        content_width = max(max(xs) - min(xs) + 90.0, min_width)
        content_height = max(max(ys) - min(ys) + 90.0, min_width)
        target_ratio = max(self.width(), 1) / max(self.height(), 1)
        if content_width / content_height < target_ratio:
            content_width = content_height * target_ratio
        else:
            content_height = content_width / target_ratio

        content_width = min(content_width, float(MAP_IMAGE_SIZE[0]))
        content_height = min(content_height, float(MAP_IMAGE_SIZE[1]))
        center_x = (min(xs) + max(xs)) / 2.0
        center_y = (min(ys) + max(ys)) / 2.0
        left = min(max(center_x - content_width / 2.0, 0.0), MAP_IMAGE_SIZE[0] - content_width)
        top = min(max(center_y - content_height / 2.0, 0.0), MAP_IMAGE_SIZE[1] - content_height)
        return QtCore.QRectF(left, top, content_width, content_height)

    def _harita_arka_plani_ciz(self, painter):
        if self._map_pixmap.isNull():
            painter.fillRect(self.rect(), QColor(27, 42, 52))
            return
        painter.drawPixmap(QtCore.QRectF(self.rect()), self._map_pixmap, self._gorunum_rect())

    def _kalibrasyon_hazirla(self):
        rows = [(p["lat"], p["lon"], 1.0) for p in MAP_CALIBRATION_POINTS]
        xs = [p["pixel"][0] for p in MAP_CALIBRATION_POINTS]
        ys = [p["pixel"][1] for p in MAP_CALIBRATION_POINTS]
        return self._least_squares_3(rows, xs), self._least_squares_3(rows, ys)

    def _least_squares_3(self, rows, values):
        normal = [[0.0 for _ in range(3)] for _ in range(3)]
        rhs = [0.0 for _ in range(3)]
        for row, value in zip(rows, values):
            for i in range(3):
                rhs[i] += row[i] * value
                for j in range(3):
                    normal[i][j] += row[i] * row[j]
        return self._solve_3x3(normal, rhs)

    def _solve_3x3(self, matrix, rhs):
        a = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]
        for col in range(3):
            pivot = max(range(col, 3), key=lambda r: abs(a[r][col]))
            a[col], a[pivot] = a[pivot], a[col]
            div = a[col][col] or 1.0
            for j in range(col, 4):
                a[col][j] /= div
            for r in range(3):
                if r == col:
                    continue
                factor = a[r][col]
                for j in range(col, 4):
                    a[r][j] -= factor * a[col][j]
        return [a[i][3] for i in range(3)]

    def _arka_plan_efekti_ciz(self, painter):
        painter.fillRect(self.rect(), QColor(7, 29, 39, 42))
        painter.setPen(QPen(QColor(255, 255, 255, 38), 1))
        step = 58
        for x in range(0, self.width(), step):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), step):
            painter.drawLine(0, y, self.width(), y)

        painter.setPen(QPen(QColor(255, 255, 255, 95), 1))
        painter.drawText(14, 24, "LIVE MISSION TRACK")

    def _bos_durum_ciz(self, painter):
        painter.setPen(QPen(QColor(255, 255, 255, 180), 1))
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        painter.drawText(self.rect(), Qt.AlignCenter, "Waiting for GPS and mission waypoints")

    def _rota_ciz(self, painter, points):
        painter.setPen(QPen(QColor(255, 193, 7, 225), 3))
        for start, end in zip(points, points[1:]):
            painter.drawLine(start, end)

    def _trail_ciz(self, painter, points):
        # Dark outline keeps the cyan trail readable on both water and land.
        painter.setPen(QPen(QColor(0, 20, 28, 210), 7))
        for start, end in zip(points, points[1:]):
            painter.drawLine(start, end)
        painter.setPen(QPen(QColor(0, 235, 255, 245), 4))
        for start, end in zip(points, points[1:]):
            painter.drawLine(start, end)
        painter.setPen(QPen(QColor(255, 255, 255, 235), 1))
        painter.setBrush(QBrush(QColor(0, 235, 255, 245)))
        for point in points[::4]:
            painter.drawEllipse(point, 3, 3)

    def _trail_durumu_ciz(self, painter):
        count = len(self._trail)
        status = "WAITING FOR MOVEMENT" if count < 2 else f"ACTIVE - {count} POINTS"
        width = 190 if count < 2 else 160
        rect = QtCore.QRect(12, 34, width, 25)
        painter.setPen(QPen(QColor(0, 235, 255, 220), 1))
        painter.setBrush(QBrush(QColor(4, 24, 31, 210)))
        painter.drawRoundedRect(rect, 5, 5)
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(rect, Qt.AlignCenter, f"TRAIL: {status}")

    def _aciklama_ciz(self, painter):
        width = 176
        rect = QtCore.QRect(max(self.width() - width - 12, 12), 10, width, 48)
        painter.setPen(QPen(QColor(255, 255, 255, 100), 1))
        painter.setBrush(QBrush(QColor(4, 24, 31, 210)))
        painter.drawRoundedRect(rect, 5, 5)
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.setPen(QPen(QColor(255, 193, 7), 4))
        painter.drawLine(rect.left() + 10, rect.top() + 15, rect.left() + 38, rect.top() + 15)
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawText(rect.left() + 46, rect.top() + 19, "IDEAL ROUTE")
        painter.setPen(QPen(QColor(0, 235, 255), 4))
        painter.drawLine(rect.left() + 10, rect.top() + 34, rect.left() + 38, rect.top() + 34)
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawText(rect.left() + 46, rect.top() + 38, "ACTUAL TRAIL")

    def _waypoint_ciz(self, painter, point, name, index):
        painter.setPen(QPen(QColor(16, 96, 62), 2))
        painter.setBrush(QBrush(QColor(46, 204, 113, 235)))
        painter.drawEllipse(point, 8, 8)

        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(int(point.x() + 11), int(point.y() - 8), f"{name or 'WP'}")

        painter.setPen(QPen(QColor(10, 54, 39), 1))
        painter.setFont(QFont("Segoe UI", 7, QFont.Bold))
        painter.drawText(int(point.x() - 4), int(point.y() + 4), str(index))

    def _arac_ciz(self, painter, point, yaw, cog=None, speed=0.0):
        try:
            yaw_deg = float(yaw)
        except (TypeError, ValueError):
            yaw_deg = 0.0
        try:
            cog_deg = float(cog)
        except (TypeError, ValueError):
            cog_deg = None
        try:
            speed_m_s = float(speed)
        except (TypeError, ValueError):
            speed_m_s = 0.0
        if speed_m_s < self.COG_MIN_SPEED_M_S:
            cog_deg = None

        painter.save()
        painter.translate(point)
        painter.rotate(yaw_deg)

        govde = QPolygonF(
            [
                QPointF(0, -20),
                QPointF(12, 12),
                QPointF(0, 7),
                QPointF(-12, 12),
            ]
        )
        painter.setPen(QPen(QColor(3, 70, 112), 3))
        painter.setBrush(QBrush(QColor(0, 210, 255, 235)))
        painter.drawPolygon(govde)

        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(QPointF(0, -17), QPointF(0, 7))
        painter.restore()

        if cog_deg is not None:
            painter.save()
            painter.translate(point)
            painter.rotate(cog_deg)
            painter.setPen(QPen(QColor(255, 193, 7, 235), 3))
            painter.setBrush(QBrush(QColor(255, 193, 7, 220)))
            painter.drawLine(QPointF(0, 0), QPointF(0, -36))
            cog_head = QPolygonF(
                [
                    QPointF(0, -44),
                    QPointF(7, -31),
                    QPointF(0, -35),
                    QPointF(-7, -31),
                ]
            )
            painter.drawPolygon(cog_head)
            painter.restore()

        painter.setPen(QPen(QColor(255, 255, 255, 225), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(point, 18, 18)

        painter.setPen(QPen(QColor(0, 210, 255), 1))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(int(point.x() + 20), int(point.y() + 5), f"{yaw_deg:.0f}°")
        if cog_deg is not None:
            painter.setPen(QPen(QColor(255, 193, 7), 1))
            painter.drawText(int(point.x() + 20), int(point.y() + 19), f"COG {cog_deg:.0f} deg")
        else:
            painter.setPen(QPen(QColor(220, 220, 220), 1))
            painter.drawText(int(point.x() + 20), int(point.y() + 19), "COG 0 deg (stopped)")


class GorevYuklemeWorker(QtCore.QObject):
    tamamlandi = QtCore.pyqtSignal(dict)
    hata = QtCore.pyqtSignal(str)

    def __init__(self, veri_sistemi, waypoints_yolu, mission_name):
        super().__init__()
        self.veri_sistemi = veri_sistemi
        self.waypoints_yolu = waypoints_yolu
        self.mission_name = mission_name

    @QtCore.pyqtSlot()
    def calistir(self):
        try:
            response = self.veri_sistemi.gorev_waypoints_yukle(
                self.waypoints_yolu,
                mission_name=self.mission_name,
            )
        except Exception as exc:
            self.hata.emit(str(exc))
            return
        self.tamamlandi.emit(response)


class GorevPlaniEkrani(QDialog):
    def __init__(self, veri_sistemi):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, "planning.ui"), self)
        self.setWindowTitle("MISSION LOAD")
        self.setWindowFlag(Qt.WindowContextHelpButtonHint, False)
        self.veri_sistemi = veri_sistemi
        self.secili_waypoints_yolu = None
        self._yukleme_thread = None
        self._yukleme_worker = None
        self._yukle_buton_metni = self.pushButton_2.text()
        self.tableWidget.setRowCount(0)
        if hasattr(self, "pushButton_4"):
            self.pushButton_4.hide()
        if hasattr(self, "pushButton_3"):
            self.pushButton_3.hide()
        self.pushButton.clicked.connect(self.dosya_sec)
        self.pushButton_2.clicked.connect(self.yukle)
        self._buton_yazilarini_sigdir()
        self.veri_sistemi.veri_guncelle.connect(self._yukleme_baglanti_durumu_guncelle)
        self._yukleme_baglanti_durumu_guncelle({})

    def _buton_yazilarini_sigdir(self):
        self.pushButton.setText("CHOOSE WAYPOINTS")
        self.pushButton_2.setText("UPLOAD WAYPOINTS")
        self._yukle_buton_metni = self.pushButton_2.text()

        buton_stili = (
            "QPushButton { color: white; border-radius: 10px; "
            "font-size: 9pt; font-weight: bold; padding: 8px 10px; }"
        )
        self.pushButton.setStyleSheet(
            buton_stili.replace("QPushButton {", "QPushButton { background-color: #3498db;")
            + "QPushButton:hover { background-color: #2980b9; }"
        )
        self.pushButton_2.setStyleSheet(
            buton_stili.replace("QPushButton {", "QPushButton { background-color: #27ae60;")
            + "QPushButton:hover { background-color: #2ecc71; }"
            + "QPushButton:disabled { background-color: #95a5a6; color: white; }"
        )
        self.pushButton.setGeometry(106, self.pushButton.y(), 250, self.pushButton.height())
        self.pushButton_2.setGeometry(106, self.pushButton_2.y(), 250, self.pushButton_2.height())
        self.pushButton.setMinimumWidth(250)
        self.pushButton_2.setMinimumWidth(250)

    def _secili_gorev_dosya_adi(self):
        for task_no in range(1, 5):
            if getattr(self, f"task{task_no}RadioButton").isChecked():
                return f"njord_task{task_no}.waypoints"
        return "njord_task1.waypoints"

    def dosya_sec(self):
        yol, _ = QFileDialog.getOpenFileName(
            self,
            "Open File",
            WAYPOINT_KLASORU,
            "QGroundControl Waypoints (*.waypoints)",
        )
        if yol:
            self.secili_waypoints_yolu = yol
            self.pushButton.setText("WAYPOINTS SELECTED")
            self.veri_sistemi.log_sinyali.emit(f"MISSION WAYPOINTS SELECTED: {os.path.basename(yol)}")
            try:
                parsed = self.veri_sistemi.gorev_waypoints_onizle(yol)
                waypoints = self._waypoints_from_response({"waypoints": parsed})
                self._tabloyu_doldur(waypoints)
                self.veri_sistemi.log_sinyali.emit(
                    f"MISSION PREVIEW READY: {len(waypoints)} waypoint(s)"
                )
            except Exception as exc:
                self.tableWidget.setRowCount(0)
                self.veri_sistemi.log_sinyali.emit(
                    f"ERROR: Mission preview failed: {exc}"
                )

    def yukle(self):
        if not self.secili_waypoints_yolu:
            self.veri_sistemi.log_sinyali.emit("ERROR: Select a mission .waypoints file first.")
            return
        if not self.veri_sistemi.arac_bagli_mi():
            self.veri_sistemi.log_sinyali.emit("ERROR: Vehicle is not connected. Mission upload blocked by GUI.")
            return
        if self._yukleme_thread is not None:
            self.veri_sistemi.log_sinyali.emit("INFO: Mission upload is already running.")
            return

        gorev_dosya_adi = self._secili_gorev_dosya_adi()
        self.pushButton_2.setEnabled(False)
        self.pushButton_2.setText("UPLOADING")
        self.veri_sistemi.log_sinyali.emit("MISSION UPLOAD STARTED: UI remains responsive.")

        thread = QtCore.QThread(self)
        worker = GorevYuklemeWorker(self.veri_sistemi, self.secili_waypoints_yolu, gorev_dosya_adi)
        worker.moveToThread(thread)
        thread.started.connect(worker.calistir)
        worker.tamamlandi.connect(self._yukleme_tamamlandi)
        worker.hata.connect(self._yukleme_hatasi)
        worker.tamamlandi.connect(worker.deleteLater)
        worker.hata.connect(worker.deleteLater)
        worker.tamamlandi.connect(thread.quit)
        worker.hata.connect(thread.quit)
        thread.finished.connect(self._yukleme_temizle)
        self._yukleme_thread = thread
        self._yukleme_worker = worker
        thread.start()

    def _yukleme_temizle(self):
        thread = self._yukleme_thread
        self._yukleme_thread = None
        self._yukleme_worker = None
        self._yukleme_baglanti_durumu_guncelle({})
        if thread is not None:
            thread.deleteLater()

    def _yukleme_hatasi(self, hata):
        self.veri_sistemi.log_sinyali.emit(f"ERROR: Mission waypoints upload failed: {hata}")

    def _yukleme_tamamlandi(self, response):
        waypoints = self._waypoints_from_response(response)
        vehicle_confirmed = bool(response.get("pixhawk_confirmed") or response.get("pixhawk_uploaded"))
        if waypoints and vehicle_confirmed:
            self._tabloyu_doldur(waypoints)
            self.veri_sistemi.gorev_noktalarini_guncelle(waypoints)
        elif waypoints:
            self._tabloyu_doldur(waypoints)
            self.veri_sistemi.log_sinyali.emit(
                "INFO: Waypoints parsed locally, but vehicle upload is not confirmed. Main map route is hidden."
            )
        else:
            self.tableWidget.setRowCount(0)
            self.veri_sistemi.gorev_noktalarini_guncelle([])

        mission_id = response.get("mission_id") or response.get("mission_name") or "backend"
        if vehicle_confirmed:
            self.veri_sistemi.log_sinyali.emit(f"MISSION WAYPOINTS SYNCED TO VEHICLE: {mission_id}")
        else:
            self.veri_sistemi.log_sinyali.emit(f"MISSION WAYPOINTS PARSED ONLY: {mission_id}")

    def _yukleme_baglanti_durumu_guncelle(self, _durum):
        if self._yukleme_thread is not None:
            return
        bagli = self.veri_sistemi.arac_bagli_mi()
        self.pushButton_2.setEnabled(bagli)
        self.pushButton_2.setText(self._yukle_buton_metni if bagli else "VEHICLE OFFLINE")

    def _waypoints_from_response(self, response):
        raw_waypoints = (
            response.get("waypoints")
            or response.get("parsed_waypoints")
            or response.get("mission_items")
            or []
        )

        waypoints = []
        for index, item in enumerate(raw_waypoints, start=1):
            if isinstance(item, dict):
                name = item.get("name") or item.get("id") or item.get("label") or f"WP_{index:02d}"
                lat = item.get("lat", item.get("latitude"))
                lon = item.get("lon", item.get("lng", item.get("longitude")))
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                name = f"WP_{index:02d}"
                lat = item[0]
                lon = item[1]
            else:
                continue

            if lat is None or lon is None:
                continue
            waypoints.append((str(name), str(lat), str(lon)))

        return waypoints

    def _tabloyu_doldur(self, waypoints):
        self.tableWidget.setRowCount(len(waypoints))

        for i, (m, lat, lon) in enumerate(waypoints):
            self.tableWidget.setItem(i, 0, QTableWidgetItem(m))
            self.tableWidget.setItem(i, 1, QTableWidgetItem(lat))
            self.tableWidget.setItem(i, 2, QTableWidgetItem(lon))

    def closeEvent(self, event):
        if self._yukleme_thread is not None:
            self.veri_sistemi.log_sinyali.emit("INFO: Mission upload continues in background.")
        super().closeEvent(event)


class NjordAnaEkran(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, "njord_redesign.ui"), self)
        self._yeni_ui_adlarini_esle()
        self._yeni_ui_layout_duzelt()
        self.setMinimumSize(720, 405)

        self._log_gecmisi = []
        self._log_dosyasi = self._log_dosyasini_hazirla()

        self.sistem = NjordVeriSistemi()
        self.harita_pencere = None
        self.plan_pencere = None
        self._ui_hazir = False
        self._last_camera_frame_time = None
        self._mode_combo_son_stil = None
        self._mode_combo_son_pixhawk_mod = None
        self._mode_combo_aday = None
        self._mode_combo_aday_zamani = 0.0
        self._vessel_status_label = None
        self._cog_label = None
        self._vessel_status_son = None
        self._vessel_status_aday = None
        self._vessel_status_aday_zamani = 0.0
        self._arac_komutlari_aktif = None
        self._son_armed_durum = False
        self._arm_buton_gorunum_durumu = None
        self._arm_durum_aday = None
        self._arm_durum_aday_zamani = 0.0
        self._arm_komut_bekliyor = False
        self._arm_komut_hedef = None
        self._arm_komut_baslangic = 0.0
        self._stop_buton_gorunum_durumu = None
        self._overview_rendered = {}
        self._kompakt_duzen_aktif = None
        self._restore_boyut_kilitli = False
        self._restore_boyut_kilidi_uygulaniyor = False
        self._restore_sabit_boyut = None

        self.sistem.veri_guncelle.connect(self.tazele)
        self.sistem.log_sinyali.connect(self.log_ekle)
        self.sistem.kamera_sinyali.connect(self.kamera_goster)
        self.sistem.waypoint_guncelle.connect(self._ana_harita_waypoint_guncelle)

        self.pushButton.clicked.connect(self.port_ac)
        self.pushButton_2.clicked.connect(self.plan_ac)

        self.pushButton_4.clicked.connect(self.arm_disarm_degistir)
        self.pushButton_8.hide()
        self.pushButton_7.clicked.connect(self.acil_durum)
        self.pushButton_6.clicked.connect(self.icra)
        self.comboBox_2.currentTextChanged.connect(self.mod_secildi)
        if self.comboBox_2.findText("HOLD") >= 0:
            self.comboBox_2.setCurrentText("HOLD")

        self._kamera_placeholder_goster("NO CAMERA SIGNAL\nWaiting for Jetson video")

        self._log_etiketleri = [
            getattr(self, "label_2", None),
            getattr(self, "label_3", None),
            getattr(self, "label_9", None),
            getattr(self, "label_10", None),
        ]
        self._log_etiketleri = [etiket for etiket in self._log_etiketleri if etiket is not None]
        for etiket in self._log_etiketleri:
            etiket.setText("")
            etiket.setStyleSheet("")

        self._ek_veri_widgetlarini_hazirla()
        self._ana_haritayi_hazirla()
        self._komut_stillerini_hazirla()
        self._olcekli_arayuz_hazirla()
        self._kamera_watchdog_timer = QtCore.QTimer(self)
        self._kamera_watchdog_timer.timeout.connect(self._kamera_watchdog_kontrol)
        self._kamera_watchdog_timer.start(1000)

        self._arm_butonlarini_sabitle()
        self._arac_komutlarini_guncelle(False)
        self._camera_auto_started = False
        self._ui_hazir = True
        QtCore.QTimer.singleShot(0, self.showMaximized)

    def _yeni_ui_adlarini_esle(self):
        eslemeler = {
            "pushButton": "BTNCONNECT",
            "pushButton_2": "BTNFILE",
            "pushButton_4": "BTNARM",
            "pushButton_5": "BTNROLL",
            "pushButton_6": "BTNMISSION",
            "pushButton_7": "BTNSTOP",
            "pushButton_8": "BTNDISARM",
            "pushButton_9": "BTNPITCH",
            "pushButton_10": "BTNYAW",
            "pushButton_11": "BTNLAT",
            "pushButton_12": "BTNLONG",
            "pushButton_wifi": "BTNWIFI",
            "comboBox_2": "CMBMODE",
            "radioButton": "RM1",
            "radioButton_2": "RM2",
            "radioButton_3": "RM3",
            "radioButton_4": "RM4",
            "label_7": "LCAMERA",
            "textEdit": "TXTCOLREG",
            "progressBar": "BATTERBAR",
            "lcdNumber_2": "LCDVOLT",
            "lcdNumber_3": "LCDSPEED",
            "label_2": "LLOG1",
            "label_3": "LLOG2",
            "label_9": "LLOG3",
            "label_10": "LLOG4",
            "label_8": "LLOGIC",
            "label_7_mainmap": "LMAINMAP",
            "textEdit_status": "TXTSTATUSLOG",
        }
        for eski_ad, yeni_ad in eslemeler.items():
            if hasattr(self, yeni_ad):
                setattr(self, eski_ad, getattr(self, yeni_ad))

    def _layouttan_cikar(self, widget):
        parent = widget.parentWidget()
        if parent is not None and parent.layout() is not None:
            parent.layout().removeWidget(widget)

    def _olcekli_arayuz_hazirla(self):
        if getattr(self, "_olcekli_arayuz_view", None) is not None:
            return

        icerik = self.takeCentralWidget()
        if icerik is None:
            return

        genislik, yukseklik = BASE_UI_SIZE
        icerik.setMinimumSize(genislik, yukseklik)
        icerik.setMaximumSize(genislik, yukseklik)
        icerik.resize(genislik, yukseklik)
        icerik.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)

        scene = QtWidgets.QGraphicsScene(self)
        scene.setSceneRect(0, 0, genislik, yukseklik)
        proxy = scene.addWidget(icerik)
        proxy.setPos(0, 0)

        view = QtWidgets.QGraphicsView(scene, self)
        view.setFrameShape(QtWidgets.QFrame.NoFrame)
        view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setAlignment(ALIGN_CENTER)
        view.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        view.setViewportUpdateMode(QtWidgets.QGraphicsView.BoundingRectViewportUpdate)
        view.setBackgroundBrush(QBrush(QColor(240, 246, 250)))

        self.setCentralWidget(view)
        self._olcekli_arayuz_icerik = icerik
        self._olcekli_arayuz_view = view
        self._olcekli_arayuz_scene = scene
        self._olcekli_arayuz_proxy = proxy
        self._arayuz_olcegini_guncelle()

    def _arayuz_olcegini_guncelle(self):
        view = getattr(self, "_olcekli_arayuz_view", None)
        if view is None:
            return

        genislik, yukseklik = BASE_UI_SIZE
        viewport = view.viewport().size()
        if viewport.width() <= 0 or viewport.height() <= 0:
            return

        olcek = min(viewport.width() / genislik, viewport.height() / yukseklik)
        view.resetTransform()
        view.scale(olcek, olcek)
        view.centerOn(genislik / 2, yukseklik / 2)
        QtCore.QTimer.singleShot(0, self._mission_yuksekligini_decision_hizasina_ayarla)
        QtCore.QTimer.singleShot(0, self._safety_yuksekligini_harita_hizasina_ayarla)
        QtCore.QTimer.singleShot(0, self._overview_yuksekligini_status_log_hizasina_ayarla)

    def _mission_yuksekligini_decision_hizasina_ayarla(self):
        if not all(hasattr(self, ad) for ad in ("groupBox_6", "groupBox_7", "groupBox_8")):
            return

        icerik = getattr(self, "_olcekli_arayuz_icerik", None)
        if icerik is None:
            return

        mission_top = self.groupBox_6.mapTo(icerik, QtCore.QPoint(0, 0)).y()
        decision_top = self.groupBox_8.mapTo(icerik, QtCore.QPoint(0, 0)).y()
        decision_bottom = decision_top + self.groupBox_8.height()
        hedef_yukseklik = decision_bottom - mission_top
        if hedef_yukseklik <= 0:
            return

        hedef_yukseklik = max(360, min(int(hedef_yukseklik), 560))
        if abs(self.groupBox_6.height() - hedef_yukseklik) <= 2:
            return

        self.groupBox_6.setMinimumHeight(hedef_yukseklik)
        self.groupBox_6.setMaximumHeight(hedef_yukseklik)
        self.groupBox_6.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        secim_yuksekligi = max(220, hedef_yukseklik - 175)
        self.groupBox_7.setMinimumHeight(secim_yuksekligi)
        self.groupBox_7.setMaximumHeight(secim_yuksekligi)
        self.groupBox_7.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        QtCore.QTimer.singleShot(0, self._overview_yuksekligini_status_log_hizasina_ayarla)

    def _safety_yuksekligini_harita_hizasina_ayarla(self):
        if not all(hasattr(self, ad) for ad in ("groupBox_bottom", "groupBox_mapMain")):
            return

        icerik = getattr(self, "_olcekli_arayuz_icerik", None)
        if icerik is None:
            return

        safety_top = self.groupBox_bottom.mapTo(icerik, QtCore.QPoint(0, 0)).y()
        map_top = self.groupBox_mapMain.mapTo(icerik, QtCore.QPoint(0, 0)).y()
        map_bottom = map_top + self.groupBox_mapMain.height()
        hedef_yukseklik = map_bottom - safety_top
        if hedef_yukseklik <= 0:
            return

        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        minimum = 170 if kompakt else 190
        hedef_yukseklik = max(minimum, min(int(hedef_yukseklik), 280))
        if abs(self.groupBox_bottom.height() - hedef_yukseklik) <= 2:
            return

        self.groupBox_bottom.setMinimumHeight(hedef_yukseklik)
        self.groupBox_bottom.setMaximumHeight(hedef_yukseklik)
        self.groupBox_bottom.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

    def _overview_yuksekligini_status_log_hizasina_ayarla(self):
        if not all(hasattr(self, ad) for ad in ("groupBox_system_overview", "groupBox_5")):
            return
        icerik = getattr(self, "_olcekli_arayuz_icerik", None)
        if icerik is None:
            return

        overview_top = self.groupBox_system_overview.mapTo(icerik, QtCore.QPoint(0, 0)).y()
        log_top = self.groupBox_5.mapTo(icerik, QtCore.QPoint(0, 0)).y()
        log_bottom = log_top + self.groupBox_5.height()
        hedef_yukseklik = int(log_bottom - overview_top)
        if hedef_yukseklik < 190:
            return

        self.groupBox_system_overview.setMinimumHeight(hedef_yukseklik)
        self.groupBox_system_overview.setMaximumHeight(hedef_yukseklik)
        self.groupBox_system_overview.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
        )

        satir_yuksekligi = max(34, min(44, int((hedef_yukseklik - 58) / 5)))
        for label in getattr(self, "_overview_values", {}).values():
            label.setMinimumHeight(satir_yuksekligi)
            label.setMaximumHeight(satir_yuksekligi)

    def _kompakt_duzen_gerekli_mi(self):
        screen = self.screen() or QtWidgets.QApplication.primaryScreen()
        if screen is not None:
            alan = screen.availableGeometry()
            if alan.width() < 1400 or alan.height() < 820:
                return True
        return self.width() < 1280 or self.height() < 760

    def _restore_boyutunu_sabitle(self):
        if self.isMaximized() or self.isFullScreen():
            return
        if self._restore_boyut_kilidi_uygulaniyor:
            return

        self._restore_boyut_kilidi_uygulaniyor = True
        try:
            boyut = self.size()
            if boyut.width() <= 0 or boyut.height() <= 0:
                return
            self.setMinimumSize(720, 405)
            self.setMaximumSize(16777215, 16777215)
            self._restore_sabit_boyut = QtCore.QSize(boyut)
            self._restore_boyut_kilitli = True
        finally:
            self._restore_boyut_kilidi_uygulaniyor = False

    def _restore_boyut_kilidini_ac(self):
        if not self._restore_boyut_kilitli:
            return
        self.setMinimumSize(720, 405)
        self.setMaximumSize(16777215, 16777215)
        self._restore_sabit_boyut = None
        self._restore_boyut_kilitli = False

    def _restore_boyutunu_gerekirse_sabitle(self):
        screen = self.screen() or QtWidgets.QApplication.primaryScreen()
        if screen is None:
            self._restore_boyutunu_sabitle()
            return

        alan = screen.availableGeometry()
        buyuk_pencere = self.width() >= int(alan.width() * 0.85) or self.height() >= int(alan.height() * 0.85)
        if buyuk_pencere:
            self._restore_boyut_kilidini_ac()
            return

        self._restore_boyutunu_sabitle()

    def _ekran_duzenini_uygula(self, zorla=False):
        kompakt = self._kompakt_duzen_gerekli_mi()
        if not zorla and self._kompakt_duzen_aktif == kompakt:
            return
        self._kompakt_duzen_aktif = kompakt
        self._ekran_modu_boylarini_uygula()
        self._batarya_paneli_tek_sutun_yap()
        self._sistem_status_dikey_yap()
        self._imu_paneli_dikey_okunur_yap()
        self._mission_secimi_dikey_yap()
        self._kucuk_ekran_minimumlarini_gevset()

    def _ekran_modu_boylarini_uygula(self):
        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        for buton in (getattr(self, "pushButton", None), getattr(self, "pushButton_2", None)):
            if buton is not None:
                buton.setMinimumHeight(34 if kompakt else 42)

        for buton in (
            getattr(self, "pushButton_6", None),
            getattr(self, "pushButton_wifi", None),
            getattr(self, "pushButton_7", None),
        ):
            if buton is not None:
                buton.setMinimumHeight(36 if kompakt else 58)

        for widget, maximum in (
            (getattr(self, "groupBox_7", None), 16777215 if kompakt else 205),
            (getattr(self, "groupBox_6", None), 220 if kompakt else 250),
            (getattr(self, "groupBox", None), 16777215 if kompakt else 360),
            (getattr(self, "groupBox_2", None), 16777215 if kompakt else 360),
            (getattr(self, "groupBox_3", None), 205 if kompakt else 235),
            (getattr(self, "groupBox_bottom", None), 16777215 if kompakt else 150),
        ):
            if widget is not None:
                widget.setMaximumHeight(maximum)

    def _kucuk_ekran_minimumlarini_gevset(self):
        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        for widget, width in (
            (getattr(self, "topVisualPanel", None), 230 if kompakt else 520),
            (getattr(self, "bottomInfoPanel", None), 570 if kompakt else 680),
            (getattr(self, "leftPanel", None), 160 if kompakt else 260),
            (getattr(self, "middlePanel", None), 160 if kompakt else 260),
            (getattr(self, "rightPanel", None), 220 if kompakt else 360),
        ):
            if widget is not None:
                widget.setMinimumWidth(width)

        for widget, size in (
            (getattr(self, "LCAMERA", None), (210, 130) if kompakt else (400, 290)),
            (getattr(self, "LMAINMAP", None), (210, 150) if kompakt else (400, 340)),
            (getattr(self, "LLOGIC", None), (120, 44) if kompakt else (170, 150)),
            (getattr(self, "TXTCOLREG", None), (180, 76) if kompakt else (300, 150)),
            (getattr(self, "TXTSTATUSLOG", None), (180, 76) if kompakt else (300, 120)),
            (getattr(self, "LALGORITHM", None), (180, 76) if kompakt else (300, 130)),
        ):
            if widget is not None:
                widget.setMinimumSize(*size)

        for widget, width in (
            (getattr(self, "LMAPGPS", None), 64 if kompakt else 110),
            (getattr(self, "LMAPSATS", None), 64 if kompakt else 110),
            (getattr(self, "LMAPDIST", None), 92 if kompakt else 210),
        ):
            if widget is not None:
                widget.setMinimumWidth(width)

        for widget in (
            getattr(self, "groupBox_4", None),
            getattr(self, "groupBox_mapMain", None),
            getattr(self, "groupBox_algorithm", None),
            getattr(self, "groupBox_8", None),
            getattr(self, "groupBox_5", None),
        ):
            if widget is not None:
                widget.setMinimumHeight(0)
                widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        for widget in (
            getattr(self, "groupBox", None),
            getattr(self, "groupBox_2", None),
            getattr(self, "groupBox_3", None),
            getattr(self, "groupBox_6", None),
            getattr(self, "groupBox_7", None),
            getattr(self, "groupBox_bottom", None),
        ):
            if widget is not None:
                widget.setMinimumHeight(0)

    def _gorsel_ust_duzeni_duzenle(self):
        if not all(hasattr(self, ad) for ad in ("topVisualPanel", "groupBox_4", "groupBox_mapMain", "groupBox_8")):
            return
        top_layout = self.topVisualPanel.layout()
        if top_layout is None:
            return

        if not hasattr(self, "topRightVisualPanel"):
            self.topRightVisualPanel = QtWidgets.QWidget(self.topVisualPanel)
            right_visual_layout = QtWidgets.QVBoxLayout(self.topRightVisualPanel)
            right_visual_layout.setContentsMargins(0, 0, 0, 0)
            right_visual_layout.setSpacing(8)
        else:
            right_visual_layout = self.topRightVisualPanel.layout()

        self._layouttan_cikar(self.groupBox_mapMain)
        self._layouttan_cikar(self.groupBox_8)

        if top_layout.indexOf(self.topRightVisualPanel) < 0:
            top_layout.addWidget(self.topRightVisualPanel)

        if right_visual_layout.indexOf(self.groupBox_mapMain) < 0:
            right_visual_layout.addWidget(self.groupBox_mapMain)
        if right_visual_layout.indexOf(self.groupBox_8) < 0:
            right_visual_layout.addWidget(self.groupBox_8)

        right_visual_layout.setStretchFactor(self.groupBox_mapMain, 4)
        right_visual_layout.setStretchFactor(self.groupBox_8, 1)
        self.topRightVisualPanel.setMinimumWidth(520)
        self.topRightVisualPanel.setMaximumWidth(780)
        self.groupBox_mapMain.setMinimumHeight(350)
        self.groupBox_8.setMinimumHeight(115)

    def _sol_gorsel_sag_bilgi_duzeni_duzenle(self):
        if not all(hasattr(self, ad) for ad in ("centralwidget", "topVisualPanel", "bottomInfoPanel")):
            return
        if hasattr(self, "mainSplitPanel"):
            return

        old_layout = self.centralwidget.layout()
        if old_layout is None:
            return

        old_layout.removeWidget(self.topVisualPanel)
        old_layout.removeWidget(self.bottomInfoPanel)

        self.mainSplitPanel = QtWidgets.QWidget(self.centralwidget)
        split_layout = QtWidgets.QHBoxLayout(self.mainSplitPanel)
        split_layout.setContentsMargins(0, 0, 0, 0)
        split_layout.setSpacing(10)

        old_layout.addWidget(self.mainSplitPanel)
        split_layout.addWidget(self.topVisualPanel)
        split_layout.addWidget(self.bottomInfoPanel)
        split_layout.setStretchFactor(self.topVisualPanel, 42)
        split_layout.setStretchFactor(self.bottomInfoPanel, 58)

        visual_layout = self.topVisualPanel.layout()
        if visual_layout is None:
            visual_layout = QtWidgets.QVBoxLayout(self.topVisualPanel)
        visual_layout.setDirection(QtWidgets.QBoxLayout.TopToBottom)
        visual_layout.setContentsMargins(0, 0, 0, 0)
        visual_layout.setSpacing(10)

        self._layouttan_cikar(self.groupBox_4)
        self._layouttan_cikar(self.groupBox_mapMain)
        if visual_layout.indexOf(self.groupBox_4) < 0:
            visual_layout.addWidget(self.groupBox_4)
        if visual_layout.indexOf(self.groupBox_mapMain) < 0:
            visual_layout.addWidget(self.groupBox_mapMain)
        visual_layout.setStretchFactor(self.groupBox_4, 45)
        visual_layout.setStretchFactor(self.groupBox_mapMain, 55)

        if hasattr(self, "topRightVisualPanel"):
            self.topRightVisualPanel.hide()

        info_layout = self.bottomInfoPanel.layout()
        if info_layout is not None and hasattr(self, "groupBox_8"):
            self._layouttan_cikar(self.groupBox_8)
            if self.rightPanel.layout() is not None and self.rightPanel.layout().indexOf(self.groupBox_8) < 0:
                self.rightPanel.layout().insertWidget(0, self.groupBox_8)

        self.topVisualPanel.setMinimumWidth(520)
        self.bottomInfoPanel.setMinimumWidth(680)
        self.LCAMERA.setMinimumSize(400, 290)
        self.LMAINMAP.setMinimumSize(400, 340)
        self.groupBox_4.setMinimumHeight(310)
        self.groupBox_mapMain.setMinimumHeight(360)

    def _safety_panelini_sola_tasi(self):
        if not all(hasattr(self, ad) for ad in ("leftPanel", "groupBox_bottom")):
            return
        left_layout = self.leftPanel.layout()
        if left_layout is None:
            return

        self._layouttan_cikar(self.groupBox_bottom)
        if left_layout.indexOf(self.groupBox_bottom) < 0:
            left_layout.addWidget(self.groupBox_bottom)
        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        safety_layout = self.groupBox_bottom.layout()
        if isinstance(safety_layout, QtWidgets.QBoxLayout):
            safety_layout.setContentsMargins(12, 24 if kompakt else 28, 12, 12)
            safety_layout.setSpacing(12 if kompakt else 16)
        self.groupBox_bottom.setMinimumHeight(170 if kompakt else 190)
        self.groupBox_bottom.setMaximumHeight(280)
        self.groupBox_bottom.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

    def _algoritma_gorsel_panelini_hazirla(self):
        if not hasattr(self, "groupBox_algorithm"):
            self.groupBox_algorithm = QtWidgets.QGroupBox("ALGORITHM / DETECTION", self.rightPanel)
            layout = QtWidgets.QVBoxLayout(self.groupBox_algorithm)
            layout.setContentsMargins(10, 20, 10, 10)
            layout.setSpacing(4)
            self.detection_graph = AlgoritmaTespitGrafigi(self.groupBox_algorithm)
            self.LALGORITHM = self.detection_graph
            layout.addWidget(self.detection_graph)

        self.groupBox_algorithm.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.LALGORITHM.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def _sag_karar_panelini_duzenle(self):
        if not all(hasattr(self, ad) for ad in ("rightPanel", "groupBox_8", "groupBox_5")):
            return
        right_layout = self.rightPanel.layout()
        if right_layout is None:
            return

        self._algoritma_gorsel_panelini_hazirla()

        for widget in (self.groupBox_algorithm, self.groupBox_8, self.groupBox_5):
            self._layouttan_cikar(widget)

        right_layout.insertWidget(0, self.groupBox_algorithm)
        right_layout.insertWidget(1, self.groupBox_8)
        right_layout.insertWidget(2, self.groupBox_5)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        for widget in (self.groupBox_algorithm, self.groupBox_8, self.groupBox_5):
            widget.setMinimumHeight(150)
            widget.setMaximumHeight(16777215)
            widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        right_layout.setStretchFactor(self.groupBox_algorithm, 1)
        right_layout.setStretchFactor(self.groupBox_8, 1)
        right_layout.setStretchFactor(self.groupBox_5, 1)

        if hasattr(self, "TXTSTATUSLOG"):
            self.TXTSTATUSLOG.setMinimumHeight(120)
            self.TXTSTATUSLOG.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.TXTSTATUSLOG.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            self.TXTSTATUSLOG.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)

        if hasattr(self, "groupBox_8") and hasattr(self, "LLOGIC") and hasattr(self, "TXTCOLREG"):
            self.groupBox_8.setTitle("DECISION LOGIC")
            decision_layout = self.groupBox_8.layout()
            if isinstance(decision_layout, QtWidgets.QGridLayout):
                self._layouttan_cikar(self.LLOGIC)
                if hasattr(self, "label_12"):
                    self._layouttan_cikar(self.label_12)
                self._layouttan_cikar(self.TXTCOLREG)
                decision_layout.addWidget(self.LLOGIC, 0, 0, 1, 2)
                if hasattr(self, "label_12"):
                    decision_layout.addWidget(self.label_12, 1, 0, 1, 2, ALIGN_CENTER)
                decision_layout.addWidget(self.TXTCOLREG, 2, 0, 1, 2)
                decision_layout.setColumnStretch(0, 1)
                decision_layout.setColumnStretch(1, 1)
                decision_layout.setRowStretch(0, 0)
                decision_layout.setRowStretch(1, 0)
                decision_layout.setRowStretch(2, 1)

            self.LLOGIC.setMinimumHeight(62)
            self.LLOGIC.setMaximumHeight(82)
            self.LLOGIC.setWordWrap(True)
            self.LLOGIC.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            self.LLOGIC.setStyleSheet(
                "QLabel { color: #111111; font-family: 'Segoe UI'; "
                "font-size: 11pt; font-weight: 600; padding: 2px; }"
            )
            self.TXTCOLREG.setMinimumHeight(110)
            self.TXTCOLREG.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.TXTCOLREG.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.TXTCOLREG.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.TXTCOLREG.setLineWrapMode(QtWidgets.QTextEdit.WidgetWidth)
            self.TXTCOLREG.document().setDocumentMargin(5)
            self.TXTCOLREG.setStyleSheet(
                "QTextEdit { background-color: #ffffff; color: #0b2239; "
                "border: 1px solid #7fb3d5; font-family: 'Segoe UI'; "
                "font-size: 8pt; font-weight: 600; padding: 2px; }"
            )

    def _batarya_paneli_tek_sutun_yap(self):
        if not hasattr(self, "groupBox_3"):
            return
        layout = self.groupBox_3.layout()
        if not isinstance(layout, QtWidgets.QGridLayout):
            return

        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        self._layout_ogelerini_temizle(layout)
        layout.setContentsMargins(8 if kompakt else 10, 18 if kompakt else 20, 8 if kompakt else 10, 8 if kompakt else 10)
        layout.setHorizontalSpacing(6 if kompakt else 8)
        layout.setVerticalSpacing(4 if kompakt else 8)

        if hasattr(self, "progressBar"):
            self.progressBar.show()
            self.progressBar.setMinimumHeight(30 if kompakt else 34)
            layout.addWidget(self.progressBar, 0, 0, 1, 2)

        total_voltage_label = getattr(self, "label_20", None)
        voltage_lcd = getattr(self, "lcdNumber_2", None)
        if total_voltage_label is not None:
            total_voltage_label.show()
            total_voltage_label.setText("TOTAL VOLTAGE")
            total_voltage_label.setMinimumHeight(22 if kompakt else 26)
            total_voltage_label.setWordWrap(False)
            layout.addWidget(total_voltage_label, 1, 0, 1, 2)
        if voltage_lcd is not None:
            voltage_lcd.show()
            voltage_lcd.setMinimumHeight(38 if kompakt else 46)
            voltage_lcd.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            layout.addWidget(voltage_lcd, 2, 0, 1, 2)

        if not hasattr(self, "LBATTERYWH"):
            self.LBATTERYWH = QtWidgets.QLabel("ENERGY LEFT: 0.0 Wh", self.groupBox_3)

        energy_style = f"font-size: {9 if kompakt else 10}pt; font-weight: bold; color: #0b2239;"
        self.LBATTERYWH.show()
        self.LBATTERYWH.setStyleSheet(energy_style)
        self.LBATTERYWH.setMinimumHeight(28 if kompakt else 34)
        self.LBATTERYWH.setWordWrap(False)
        self.LBATTERYWH.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        layout.addWidget(self.LBATTERYWH, 3, 0, 1, 2)
        for row in range(4):
            layout.setRowStretch(row, 0)

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        self.groupBox_3.setMaximumHeight(205 if kompakt else 235)
        self.groupBox_3.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

    def _layout_ogelerini_temizle(self, layout):
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            child_layout = item.layout()
            if widget is not None:
                layout.removeWidget(widget)
            elif child_layout is not None:
                self._layout_ogelerini_temizle(child_layout)

    def _sistem_status_dikey_yap(self):
        if not hasattr(self, "groupBox") or self.groupBox.layout() is None:
            return
        layout = self.groupBox.layout()
        self._layout_ogelerini_temizle(layout)
        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        layout.setContentsMargins(10 if kompakt else 14, 20 if kompakt else 26, 10 if kompakt else 14, 10 if kompakt else 16)
        layout.setSpacing(6 if kompakt else 14)

        if kompakt:
            connect_button = getattr(self, "pushButton", None)
            if connect_button is not None:
                layout.addWidget(connect_button)
                connect_button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

            if self._vessel_status_label is not None:
                layout.addWidget(self._vessel_status_label)
                self._vessel_status_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

            arm_layout = QtWidgets.QHBoxLayout()
            arm_layout.setContentsMargins(0, 0, 0, 0)
            arm_layout.setSpacing(6)
            for widget in (getattr(self, "pushButton_4", None),):
                if widget is not None:
                    arm_layout.addWidget(widget)
                    widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            layout.addLayout(arm_layout)

            if hasattr(self, "comboBox_2"):
                layout.addWidget(self.comboBox_2)
                self.comboBox_2.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        else:
            for widget in (
                getattr(self, "pushButton", None),
                self._vessel_status_label,
                getattr(self, "pushButton_4", None),
                getattr(self, "comboBox_2", None),
            ):
                if widget is None:
                    continue
                layout.addWidget(widget)
                widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
                if widget is self._vessel_status_label:
                    layout.addSpacing(12)

        if hasattr(self, "pushButton"):
            self.pushButton.setMinimumHeight(38 if kompakt else 54)
        if self._vessel_status_label is not None:
            self._vessel_status_label.setMinimumHeight(42 if kompakt else 62)
            self._vessel_status_label.setMaximumHeight(54 if kompakt else 86)
        if getattr(self, "pushButton_4", None) is not None:
            self.pushButton_4.setMinimumHeight(36 if kompakt else 50)
        if getattr(self, "pushButton_8", None) is not None:
            self.pushButton_8.hide()
        if hasattr(self, "comboBox_2"):
            self.comboBox_2.setMinimumHeight(36 if kompakt else 50)

    def _imu_paneli_dikey_okunur_yap(self):
        if not hasattr(self, "groupBox_2"):
            return
        layout = self.groupBox_2.layout()
        if not isinstance(layout, QtWidgets.QGridLayout):
            return

        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        self._layout_ogelerini_temizle(layout)
        layout.setContentsMargins(8 if kompakt else 12, 18 if kompakt else 22, 8 if kompakt else 12, 8 if kompakt else 12)
        layout.setHorizontalSpacing(6 if kompakt else 8)
        layout.setVerticalSpacing(4 if kompakt else 8)

        roll = getattr(self, "pushButton_5", None)
        pitch = getattr(self, "pushButton_9", None)
        yaw = getattr(self, "pushButton_10", None)
        lat_title = getattr(self, "label_lat_title", None)
        lon_title = getattr(self, "label_lon_title", None)
        speed_title = getattr(self, "label_speed_title", None)
        lat_value = getattr(self, "pushButton_11", None)
        lon_value = getattr(self, "pushButton_12", None)
        speed_value = getattr(self, "lcdNumber_3", None)

        for row, widget in enumerate((roll, pitch, yaw)):
            if widget is not None:
                layout.addWidget(widget, row, 0, 1, 1 if kompakt else 2)
                widget.setMinimumHeight(30 if kompakt else 38)
                widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        pairs = (
            (lat_title, lat_value),
            (lon_title, lon_value),
            (speed_title, speed_value),
        )
        start_row = 3
        for offset, (title, value) in enumerate(pairs):
            row = start_row + offset
            if title is not None:
                layout.addWidget(title, offset if kompakt else row, 1 if kompakt else 0)
                title.setMinimumHeight(28 if kompakt else 34)
                title.setStyleSheet("font-size: 10pt; font-weight: bold; color: #0b2239;")
            if value is not None:
                layout.addWidget(value, offset if kompakt else row, 2 if kompakt else 1)
                value.setMinimumHeight(30 if kompakt else 38)
                value.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        if self._cog_label is not None:
            layout.addWidget(self._cog_label, 3 if kompakt else start_row + len(pairs), 0, 1, 3 if kompakt else 2, ALIGN_CENTER)
            self._cog_label.setMinimumSize(100 if kompakt else 150, 28 if kompakt else 36)
            self._cog_label.setMaximumHeight(34 if kompakt else 42)

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1 if kompakt else 2)
        layout.setColumnStretch(2, 2 if kompakt else 0)

    def _mission_secimi_dikey_yap(self):
        if not hasattr(self, "groupBox_7") or self.groupBox_7.layout() is None:
            return
        layout = self.groupBox_7.layout()
        self._layout_ogelerini_temizle(layout)
        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        layout.setContentsMargins(10 if kompakt else 14, 18 if kompakt else 20, 10 if kompakt else 14, 10 if kompakt else 14)
        layout.setSpacing(6 if kompakt else 8)
        if isinstance(layout, QtWidgets.QBoxLayout):
            layout.setDirection(QtWidgets.QBoxLayout.TopToBottom)

        for radio in (self.radioButton, self.radioButton_2, self.radioButton_3, self.radioButton_4):
            layout.addWidget(radio)
            radio.setMinimumHeight(26 if kompakt else 34)
            radio.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.groupBox_7.setMinimumHeight(245 if kompakt else 285)
        self.groupBox_7.setMaximumHeight(245 if kompakt else 285)
        self.groupBox_7.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def _yeni_ui_layout_duzelt(self):
        if hasattr(self, "topVisualPanel") and hasattr(self, "bottomInfoPanel"):
            self._safety_panelini_sola_tasi()
            self._sol_gorsel_sag_bilgi_duzeni_duzenle()
            self._sag_karar_panelini_duzenle()
            self._batarya_paneli_tek_sutun_yap()
            self._mission_secimi_dikey_yap()

            main_layout = getattr(self, "verticalLayout_main", None)
            if main_layout is not None:
                main_layout.setStretchFactor(getattr(self, "mainSplitPanel", self.topVisualPanel), 1)

            if self.topVisualPanel.layout() is not None:
                self.topVisualPanel.layout().setStretchFactor(self.groupBox_4, 45)
                self.topVisualPanel.layout().setStretchFactor(self.groupBox_mapMain, 55)

            self.topVisualPanel.setMinimumHeight(0)
            self.groupBox_4.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.groupBox_mapMain.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.LCAMERA.setMinimumSize(400, 290)
            self.LMAINMAP.setMinimumSize(400, 340)

        if hasattr(self, "leftPanel") and hasattr(self, "middlePanel") and hasattr(self, "rightPanel"):
            panel_layout = getattr(self, "horizontalLayout", None)
            if panel_layout is not None:
                panel_layout.setContentsMargins(0, 0, 0, 0)
                panel_layout.setSpacing(8)
                panel_layout.setStretch(0, 27)
                panel_layout.setStretch(1, 27)
                panel_layout.setStretch(2, 46)
                panel_layout.setStretchFactor(self.leftPanel, 27)
                panel_layout.setStretchFactor(self.middlePanel, 27)
                panel_layout.setStretchFactor(self.rightPanel, 46)

            self.leftPanel.setMinimumWidth(260)
            self.middlePanel.setMinimumWidth(260)
            self.rightPanel.setMinimumWidth(360)
            for panel in (self.leftPanel, self.middlePanel, self.rightPanel):
                panel_layout_inner = panel.layout()
                if isinstance(panel_layout_inner, QtWidgets.QBoxLayout):
                    panel_layout_inner.setAlignment(Qt.AlignTop)
                    panel_layout_inner.setContentsMargins(0, 0, 0, 0)
                    panel_layout_inner.setSpacing(8)
            for panel in (self.leftPanel, self.middlePanel, self.rightPanel):
                panel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        elif hasattr(self, "leftPanel") and hasattr(self, "rightPanel"):
            panel_layout = getattr(self, "horizontalLayout", None)
            if panel_layout is not None:
                panel_layout.setStretch(0, 35)
                panel_layout.setStretch(1, 65)
                panel_layout.setStretchFactor(self.leftPanel, 35)
                panel_layout.setStretchFactor(self.rightPanel, 65)

            self.leftPanel.setMinimumWidth(420)
            self.rightPanel.setMinimumWidth(760)
            for panel in (self.leftPanel, self.rightPanel):
                panel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        if (
            hasattr(self, "groupBox_4")
            and hasattr(self, "LCAMERA")
            and hasattr(self, "groupBox_8")
            and self.groupBox_8.parentWidget() == self.groupBox_4
        ):
            right_layout = getattr(self, "rightPanel", None)
            if right_layout is not None and self.rightPanel.layout() is not None and hasattr(self, "groupBox_5"):
                self.rightPanel.layout().setStretchFactor(self.groupBox_4, 4)
                self.rightPanel.layout().setStretchFactor(self.groupBox_5, 1)

            layout = self.groupBox_4.layout()
            if layout is None:
                layout = QtWidgets.QVBoxLayout(self.groupBox_4)
                layout.setContentsMargins(10, 24, 10, 10)
                layout.setSpacing(10)
                layout.addWidget(self.LCAMERA, 4)
                layout.addWidget(self.groupBox_8, 4)
            else:
                layout.setStretchFactor(self.LCAMERA, 4)
                layout.setStretchFactor(self.groupBox_8, 4)

            self.LCAMERA.setMinimumSize(320, 180)
            self.LCAMERA.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.groupBox_8.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        if hasattr(self, "groupBox_8") and hasattr(self, "LLOGIC") and hasattr(self, "TXTCOLREG"):
            decision_layout = self.groupBox_8.layout()
            if decision_layout is None:
                decision_layout = QtWidgets.QGridLayout(self.groupBox_8)
                decision_layout.setContentsMargins(10, 24, 10, 10)
                decision_layout.setSpacing(6)
                decision_layout.addWidget(self.LLOGIC, 0, 0, 2, 1)
                if hasattr(self, "label_12"):
                    decision_layout.addWidget(self.label_12, 0, 1)
                decision_layout.addWidget(self.TXTCOLREG, 1, 1)
                decision_layout.setColumnStretch(0, 1)
                decision_layout.setColumnStretch(1, 2)
                decision_layout.setRowStretch(1, 1)
            else:
                decision_layout.setColumnStretch(0, 1)
                decision_layout.setColumnStretch(1, 2)
                decision_layout.setRowStretch(1, 1)

            self.LLOGIC.setMinimumSize(170, 150)
            self.LLOGIC.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.TXTCOLREG.setMinimumSize(300, 150)
            self.TXTCOLREG.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.TXTCOLREG.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.TXTCOLREG.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.TXTCOLREG.setLineWrapMode(QtWidgets.QTextEdit.WidgetWidth)
            if hasattr(self, "label_12"):
                self.label_12.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        self._sag_karar_panelini_duzenle()

    def showEvent(self, event):
        super().showEvent(event)
        if not self._camera_auto_started:
            self._camera_auto_started = True
            self.sistem.kamera_oto_baslat()

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() == QtCore.QEvent.WindowStateChange and getattr(self, "_ui_hazir", False):
            if self.isMaximized() or self.isFullScreen():
                self._restore_boyut_kilidini_ac()
            else:
                QtCore.QTimer.singleShot(0, self._restore_boyutunu_gerekirse_sabitle)

    def kamera_goster(self, image):
        if image is None:
            self._last_camera_frame_time = None
            self._kamera_placeholder_goster("NO CAMERA SIGNAL\nWaiting for video signal")
            return

        self._last_camera_frame_time = time.time()
        pixmap = QPixmap.fromImage(image)
        self.label_7.setText("")
        self.label_7.setStyleSheet("background-color: #000000;")

        hedef = self.label_7.size()
        if hedef.width() > 0 and hedef.height() > 0:
            pixmap = pixmap.scaled(hedef, KEEP_ASPECT, SMOOTH)

        self.label_7.setPixmap(pixmap)
        self.label_7.setAlignment(ALIGN_CENTER)

    def _kamera_watchdog_kontrol(self):
        if self._last_camera_frame_time is None:
            return
        if time.time() - self._last_camera_frame_time > 3.0:
            self._kamera_placeholder_goster("NO CAMERA SIGNAL\nWaiting for video signal")
            self._last_camera_frame_time = None

    def _kamera_placeholder_goster(self, mesaj):
        self.label_7.setPixmap(QPixmap())
        self.label_7.clear()
        self.label_7.setText(mesaj)
        self.label_7.setStyleSheet(
            "background-color: #2c3e50; color: #f39c12; "
            "font-weight: bold; border: 2px dashed #f39c12;"
        )
        self.label_7.setAlignment(ALIGN_CENTER)

    def port_ac(self):
        durum = self.sistem.durum_al()
        if durum.get("baglanti") or durum.get("telemetry_lost") or getattr(self.sistem, "connection", None):
            self.sistem.baglanti_kes()
            return

        pencere = PortEkrani(self.sistem)
        pencere.exec_()

    def plan_ac(self):
        self.plan_pencere = GorevPlaniEkrani(self.sistem)
        self.plan_pencere.show()

    def _ana_haritayi_hazirla(self):
        self._ana_harita_katmani = None
        if not hasattr(self, "LMAINMAP"):
            return

        map_path = os.path.join(GORSEL_KLASORU, MAP_IMAGE_FILE)
        map_pixmap = QPixmap()
        if os.path.exists(map_path):
            map_pixmap = QPixmap(map_path)
        self.LMAINMAP.setPixmap(QPixmap())
        self.LMAINMAP.setScaledContents(False)
        self.LMAINMAP.setText("")
        self.LMAINMAP.setStyleSheet("background-color: #1b2a34; border: 1px solid #7fb3d5;")

        self._ana_harita_katmani = HaritaCizimKatmani(self.LMAINMAP, map_pixmap)
        self._ana_harita_katmani.setGeometry(self.LMAINMAP.rect())
        self._ana_harita_katmani.raise_()
        self._ana_harita_katmani.set_waypoints(self.sistem.gorev_noktalarini_al())

    def _ana_harita_waypoint_guncelle(self, waypoints):
        if self._ana_harita_katmani is not None:
            self._ana_harita_katmani.set_waypoints(waypoints)
        if hasattr(self, "_overview_values"):
            self._system_overview_guncelle(self.sistem.durum_al())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if (
            getattr(self, "_ui_hazir", False)
            and self._restore_boyut_kilitli
            and self._restore_sabit_boyut is not None
            and not self._restore_boyut_kilidi_uygulaniyor
            and not self.isMaximized()
            and not self.isFullScreen()
            and (
                event.size().width() > self._restore_sabit_boyut.width() + 80
                or event.size().height() > self._restore_sabit_boyut.height() + 80
            )
        ):
            self._restore_boyut_kilidini_ac()

        if (
            getattr(self, "_ui_hazir", False)
            and self._restore_boyut_kilitli
            and self._restore_sabit_boyut is not None
            and not self._restore_boyut_kilidi_uygulaniyor
            and not self.isMaximized()
            and not self.isFullScreen()
            and event.size() != self._restore_sabit_boyut
        ):
            self._restore_boyut_kilidi_uygulaniyor = True
            try:
                self.resize(self._restore_sabit_boyut)
            finally:
                self._restore_boyut_kilidi_uygulaniyor = False
            return

        self._arayuz_olcegini_guncelle()
        if getattr(self, "_ana_harita_katmani", None) is not None and hasattr(self, "LMAINMAP"):
            self._ana_harita_katmani.setGeometry(self.LMAINMAP.rect())

    def _ek_veri_widgetlarini_hazirla(self):
        self._vessel_status_label = getattr(self, "LVESSELSTATUS", None)
        if self._vessel_status_label is None:
            self._vessel_status_label = QtWidgets.QLabel("OUT OF CONTROL", self.groupBox)
            if self.groupBox.layout() is not None:
                self.groupBox.layout().addWidget(self._vessel_status_label)
        self._vessel_status_label.setAlignment(ALIGN_CENTER)
        self._vessel_status_label.setMinimumHeight(38)
        self._vessel_status_label.setMaximumHeight(58)
        self._vessel_status_label.setWordWrap(True)
        self._vessel_status_label.setText("OUT OF CONTROL")
        self._vessel_status_label.setStyleSheet(self._vessel_status_stili("#c0392b"))
        self._vessel_status_son = ("OUT OF CONTROL", "#c0392b")
        self._sistem_status_dikey_yap()
        self._system_overview_hazirla()

        self._cog_label = getattr(self, "LCOG", None)
        if self._cog_label is None:
            self._cog_label = QtWidgets.QLabel("COG: 0.0°", self.groupBox_2)
        if self.groupBox_2.layout() is not None:
            self.groupBox_2.layout().addWidget(self._cog_label, 3, 1, 1, 1, ALIGN_CENTER)
        self._cog_label.setAlignment(ALIGN_CENTER)
        self._cog_label.setMinimumSize(100, 28)
        self._cog_label.setMaximumHeight(34)
        self._cog_label.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self._cog_label.setStyleSheet(
            "QLabel { background-color: #f8fcff; color: #0b2239; "
            "border: 2px solid #00a8e8; border-radius: 6px; "
            "font-size: 10pt; font-weight: bold; padding: 3px; }"
        )
        self._imu_paneli_dikey_okunur_yap()

        map_status_stili = (
            "QLabel { background-color: #f8fcff; color: #0b2239; "
            "border: 2px solid #00a8e8; border-radius: 6px; "
            "font-size: 10pt; font-weight: bold; padding: 3px; }"
        )
        for ad in ("LMAPGPS", "LMAPSATS", "LMAPDIST"):
            widget = getattr(self, ad, None)
            if widget is not None:
                widget.setStyleSheet(map_status_stili)
                widget.setMinimumHeight(34)
                widget.setMaximumHeight(34)
                widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        map_layout = getattr(self, "verticalLayout_map", None)
        if map_layout is not None:
            # The map consumes all spare vertical space; the three telemetry
            # indicators remain a compact status strip below it.
            map_layout.setStretch(0, 1)
            map_layout.setStretch(1, 0)

        if hasattr(self, "label_20"):
            self.label_20.setStyleSheet("font-size: 11pt; font-weight: bold; color: #0b2239;")

        self.textEdit.setPlainText("DETECTIONS: 0 | FRAME: --\nStatus: No active detection")

    def _system_overview_hazirla(self):
        if not hasattr(self, "middlePanel") or self.middlePanel.layout() is None:
            return

        self.groupBox_system_overview = QtWidgets.QGroupBox("SYSTEM OVERVIEW", self.middlePanel)
        overview_layout = QtWidgets.QGridLayout(self.groupBox_system_overview)
        overview_layout.setContentsMargins(9, 24, 9, 9)
        overview_layout.setVerticalSpacing(5)
        overview_layout.setColumnStretch(0, 1)

        rows = (
            ("connection", "VEHICLE CONNECTION"),
            ("arm", "ARM STATUS"),
            ("mode", "OPERATING MODE"),
            ("waypoints", "SELECTED TASK"),
            ("mission", "MISSION STATUS"),
        )
        self._overview_values = {}
        self._overview_titles = {}
        self._overview_rendered = {}
        for row, (key, title) in enumerate(rows):
            value_label = QtWidgets.QLabel(f"{title}: --", self.groupBox_system_overview)
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setWordWrap(True)
            value_label.setMinimumHeight(36)
            value_label.setMaximumHeight(36)
            value_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            overview_layout.addWidget(value_label, row, 0)
            self._overview_values[key] = value_label
            self._overview_titles[key] = title

        middle_layout = self.middlePanel.layout()
        self._layouttan_cikar(self.groupBox_system_overview)
        mission_index = middle_layout.indexOf(getattr(self, "groupBox_6", None))
        middle_layout.insertWidget(mission_index + 1 if mission_index >= 0 else middle_layout.count(), self.groupBox_system_overview)
        middle_layout.setSpacing(8)
        middle_layout.setStretchFactor(self.groupBox_system_overview, 0)
        self.groupBox_system_overview.setMinimumHeight(230)
        self.groupBox_system_overview.setMaximumHeight(230)
        self.groupBox_system_overview.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Fixed,
        )
        self._system_overview_guncelle(self.sistem.durum_al())

    def _overview_degerini_ayarla(self, key, text, color):
        label = getattr(self, "_overview_values", {}).get(key)
        if label is None:
            return
        title = getattr(self, "_overview_titles", {}).get(key, key.upper())
        rendered = (str(title), str(text), str(color))
        if getattr(self, "_overview_rendered", {}).get(key) == rendered:
            return
        self._overview_rendered[key] = rendered
        safe_title = html.escape(str(title))
        safe_text = html.escape(str(text))
        label.setText(
            f"<span style='color:#111111; font-weight:700;'>{safe_title}:</span> "
            f"<span style='color:{color}; font-weight:900;'>{safe_text}</span>"
        )
        label.setStyleSheet(
            "QLabel { background-color: #f8fcff; color: #111111; border: 1px solid %s; "
            "border-radius: 6px; font-size: 9pt; font-weight: 800; padding: 2px 6px; }"
            % color
        )

    def _system_overview_guncelle(self, d):
        connected = bool(
            d.get("baglanti")
            and d.get("link_ok")
            and d.get("heartbeat_seen")
            and not d.get("telemetry_lost")
        )
        armed = bool(d.get("armed"))
        mode = str(d.get("mod", "UNKNOWN") or "UNKNOWN").upper()
        mission = str(d.get("active_mission") or "--")
        waypoint_count = len(self.sistem.gorev_noktalarini_al())

        task_name = mission
        if mission.upper() in ("M1", "M2", "M3", "M4"):
            task_name = f"TASK {mission[-1]}"
        elif mission.lower() in ("task1", "task2", "task3", "task4"):
            task_name = f"TASK {mission[-1]}"
        elif mission == "--":
            selected_radios = (
                (getattr(self, "radioButton", None), "TASK 1"),
                (getattr(self, "radioButton_2", None), "TASK 2"),
                (getattr(self, "radioButton_3", None), "TASK 3"),
                (getattr(self, "radioButton_4", None), "TASK 4"),
            )
            task_name = next(
                (name for radio, name in selected_radios if radio is not None and radio.isChecked()),
                "NOT SELECTED",
            )

        self._overview_degerini_ayarla(
            "connection", "OK" if connected else "NO LINK", "#1e8449" if connected else "#c0392b"
        )
        if connected:
            self._overview_degerini_ayarla(
                "arm",
                "ARMED" if armed else "DISARMED",
                "#c0392b" if armed else "#2980b9",
            )
        else:
            self._overview_degerini_ayarla("arm", "NO DATA", "#7f8c8d")
        mode_color = "#1e8449" if mode in ("AUTO", "GUIDED") else "#b9770e" if connected else "#7f8c8d"
        self._overview_degerini_ayarla("mode", mode, mode_color)
        self._overview_degerini_ayarla(
            "waypoints", f"{task_name} / {waypoint_count} WP" if waypoint_count else task_name,
            "#1e8449" if waypoint_count and task_name != "NOT SELECTED" else "#7f8c8d",
        )

        decision = d.get("mission_decision")
        if not connected:
            mission_status, mission_color = "WAITING FOR VEHICLE", "#c0392b"
        elif isinstance(decision, dict):
            mission_status = (
                f"{decision.get('stage') or 'ACTIVE'} / "
                f"{float(decision.get('progress_percent', 0.0)):.0f}%"
            )
            mission_color = (
                "#c0392b"
                if str(decision.get("stage", "")).upper() == "FAILSAFE"
                else "#b9770e"
                if decision.get("collision_risk") is True
                else "#1e8449"
            )
        elif not waypoint_count:
            mission_status, mission_color = "NOT LOADED", "#b9770e"
        elif armed and mode in ("AUTO", "GUIDED") and mission != "--":
            mission_status, mission_color = "IN PROGRESS", "#1e8449"
        elif mission != "--":
            mission_status, mission_color = "READY", "#2980b9"
        else:
            mission_status, mission_color = "LOADED", "#2980b9"
        self._overview_degerini_ayarla("mission", mission_status, mission_color)

        self.label_8.setText("Mission state: Waiting for telemetry")

    def _vessel_status_stili(self, renk):
        return (
            f"QLabel {{ background-color: {renk}; color: white; border-radius: 8px; "
            "font-size: 11pt; font-weight: bold; padding: 6px; }}"
        )

    def _komut_stillerini_hazirla(self):
        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        self._komut_buton_stili = (
            "QPushButton { background-color: #34495e; color: white; "
            "border-radius: 12px; font-weight: bold; padding: 8px; "
            "border: 1px solid #2c3e50; }"
            "QPushButton:hover { background-color: #2c3e50; }"
            "QPushButton:pressed { background-color: #1a252f; }"
            "QPushButton:disabled { background-color: #95a5a6; color: #ecf0f1; "
            "border: 1px solid #7f8c8d; }"
        )
        self._mode_combo_stili = (
            "QComboBox { background-color: #34495e; color: white; "
            "border-radius: 12px; font-weight: bold; padding: 8px 30px 8px 12px; "
            "border: 1px solid #2c3e50; }"
            "QComboBox::drop-down { subcontrol-origin: padding; "
            "subcontrol-position: top right; width: 28px; border-left: 1px solid #2c3e50; }"
            "QComboBox::down-arrow { image: none; width: 0; height: 0; "
            "border-left: 6px solid transparent; border-right: 6px solid transparent; "
            "border-top: 8px solid white; margin-top: 2px; }"
            "QComboBox QAbstractItemView { background-color: #1a1a1a; color: #ecf0f1; "
            "selection-background-color: #3498db; selection-color: white; outline: none; }"
            "QComboBox:disabled { background-color: #95a5a6; color: #ecf0f1; "
            "border: 1px solid #7f8c8d; }"
        )
        self._veri_gostergesi_stili = (
            "QPushButton { background-color: #f8fcff; color: #0b2239; "
            "border: 2px solid #00a8e8; border-radius: 6px; "
            "font-size: 11pt; font-weight: bold; padding: 6px; }"
        )
        self._lcd_stili = (
            "QLCDNumber { color: #00D2FF; border: 2px solid #00a8e8; "
            "border-radius: 5px; background-color: #f8fcff; }"
        )
        self._batarya_stili = (
            "QProgressBar { border: 1px solid #7fb3d5; border-radius: 4px; "
            "background-color: #eef6fb; text-align: center; font-weight: bold; }"
            "QProgressBar::chunk { background-color: #3498db; border-radius: 3px; }"
        )
        self._acil_stop_stili = (
            "QPushButton { background-color: #b71c1c; color: white; "
            "border-radius: 0px; font-size: 11pt; font-weight: 900; padding: 8px; "
            "border: 3px solid #4a0000; }"
            "QPushButton:hover { background-color: #c62828; border-color: #210000; }"
            "QPushButton:pressed { background-color: #7f0000; "
            "border: 3px solid #210000; }"
            "QPushButton:disabled { background-color: #666666; color: #dddddd; "
            "border: 3px solid #333333; }"
        )
        self._safe_start_stili = (
            "QPushButton { background-color: #1b5e20; color: white; "
            "border-radius: 0px; font-size: 11pt; font-weight: 900; padding: 8px; "
            "border: 3px solid #073b0b; }"
            "QPushButton:hover { background-color: #237a2a; border-color: #032306; }"
            "QPushButton:pressed { background-color: #0d4212; "
            "border: 3px solid #032306; }"
            "QPushButton:disabled { background-color: #666666; color: #dddddd; "
            "border: 3px solid #333333; }"
        )
        self._relay_pending_stili = (
            "QPushButton { background-color: #b26a00; color: white; "
            "border-radius: 0px; font-size: 11pt; font-weight: 900; padding: 8px; "
            "border: 3px solid #5f3700; }"
        )

        for buton in (
            self.pushButton,
            self.pushButton_2,
            self.pushButton_6,
        ):
            buton.setStyleSheet(self._komut_buton_stili)
        for buton in (
            self.pushButton_5,
            self.pushButton_9,
            self.pushButton_10,
            self.pushButton_11,
            self.pushButton_12,
        ):
            buton.setStyleSheet(self._veri_gostergesi_stili)

        for lcd in (
            getattr(self, "lcdNumber_2", None),
            getattr(self, "lcdNumber_3", None),
        ):
            if lcd is not None:
                lcd.setStyleSheet(self._lcd_stili)

        self.pushButton.setMinimumHeight(38 if kompakt else 48)
        self.pushButton.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.pushButton_2.setMinimumHeight(34 if kompakt else 42)
        self.pushButton_2.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        self.textEdit.setStyleSheet(
            "QTextEdit { font-family: 'Segoe UI'; font-size: 8pt; font-weight: 600; "
            "color: #0b2239; background-color: #ffffff; "
            "border: 1px solid #7fb3d5; padding: 2px; }"
        )

        for etiket in self._log_etiketleri:
            etiket.setMinimumHeight(28)
            etiket.setStyleSheet("font-size: 12pt; font-weight: bold; color: #1f618d;")

        self.progressBar.setStyleSheet(self._batarya_stili)
        self.pushButton_7.setStyleSheet(self._acil_stop_stili)
        self.pushButton_6.setStyleSheet(self._komut_buton_stili)
        self.comboBox_2.setStyleSheet(self._mode_combo_stili)

        if hasattr(self, "groupBox_7"):
            self.groupBox_7.setMinimumHeight(245 if kompakt else 285)
            self.groupBox_7.setMaximumHeight(245 if kompakt else 285)
            self.groupBox_7.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        radio_stili = (
            "QRadioButton { font-size: 12pt; font-weight: bold; color: #0b2239; "
            "spacing: 10px; padding: 6px; }"
            "QRadioButton::indicator { width: 20px; height: 20px; }"
            "QRadioButton::indicator:unchecked { border: 2px solid #5dade2; "
            "border-radius: 10px; background-color: #ffffff; }"
            "QRadioButton::indicator:checked { border: 2px solid #1f618d; "
            "border-radius: 10px; background-color: #3498db; }"
        )
        for radio in (self.radioButton, self.radioButton_2, self.radioButton_3, self.radioButton_4):
            radio.setMinimumHeight(26 if kompakt else 34)
            radio.setStyleSheet(radio_stili)
            radio.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        if hasattr(self, "groupBox_6"):
            self.groupBox_6.setMaximumHeight(445 if kompakt else 485)
            self.groupBox_6.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

        if hasattr(self, "groupBox"):
            self.groupBox.setMaximumHeight(360)
            self.groupBox.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

        if hasattr(self, "groupBox_2"):
            self.groupBox_2.setMaximumHeight(360)
            self.groupBox_2.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

        if hasattr(self, "groupBox_3"):
            self.groupBox_3.setMaximumHeight(205 if kompakt else 235)
            self.groupBox_3.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

        if hasattr(self, "middlePanel") and self.middlePanel.layout() is not None:
            orta_layout = self.middlePanel.layout()
            for panel in (
                getattr(self, "leftPanel", None),
                getattr(self, "middlePanel", None),
                getattr(self, "rightPanel", None),
            ):
                panel_layout_inner = panel.layout() if panel is not None else None
                if isinstance(panel_layout_inner, QtWidgets.QBoxLayout):
                    panel_layout_inner.setAlignment(Qt.AlignTop)
                    panel_layout_inner.setContentsMargins(0, 0, 0, 0)
                    panel_layout_inner.setSpacing(8)
            if hasattr(self, "groupBox_3"):
                orta_layout.setStretchFactor(self.groupBox_3, 0)
            if hasattr(self, "groupBox_6"):
                orta_layout.setStretchFactor(self.groupBox_6, 0)
                self.groupBox_6.setMaximumHeight(445 if kompakt else 485)
                self.groupBox_6.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
            if hasattr(self, "groupBox_system_overview"):
                orta_layout.setStretchFactor(self.groupBox_system_overview, 0)
                self.groupBox_system_overview.setSizePolicy(
                    QtWidgets.QSizePolicy.Expanding,
                    QtWidgets.QSizePolicy.Fixed,
                )
            if hasattr(self, "pushButton_wifi"):
                orta_layout.setStretchFactor(self.pushButton_wifi, 0)
            if hasattr(self, "pushButton_7"):
                orta_layout.setStretchFactor(self.pushButton_7, 0)

        self.pushButton_6.setMinimumHeight(40 if kompakt else 50)
        self.pushButton_6.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.pushButton_2.setMaximumWidth(16777215)
        self.pushButton_6.setMaximumWidth(16777215)
        for buton in (self.pushButton_wifi, self.pushButton_7):
            buton_yukseklik = 46 if kompakt else 56
            buton.setMinimumHeight(buton_yukseklik)
            buton.setMaximumHeight(buton_yukseklik)
            buton.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        self._kompakt_duzen_aktif = False

    @staticmethod
    def _detection_metni(d):
        detections = [item for item in d.get("detections", []) if isinstance(item, dict)]
        if not detections:
            return "Active detections: 0 | Camera frame: --\nNo active detection"

        lines = [
            f"Active detections: {len(detections)} | "
            f"Camera frame: {detections[-1].get('frame_id', '--')}",
        ]
        visible_detections = detections[:4]
        for index, detection in enumerate(visible_detections, start=1):
            try:
                confidence_text = f"{float(detection['confidence']) * 100.0:.0f}%"
            except (KeyError, TypeError, ValueError):
                confidence_text = "--"
            try:
                depth_text = f"{float(detection['depth']):.2f} m"
            except (KeyError, TypeError, ValueError):
                depth_text = "--"
            try:
                angle_text = f"{float(detection['angle']):+.1f}°"
            except (KeyError, TypeError, ValueError):
                angle_text = "--"
            try:
                position_text = (
                    f"{float(detection['lat']):.7f}, "
                    f"{float(detection['lon']):.7f}"
                )
            except (KeyError, TypeError, ValueError):
                position_text = "--"

            lines.extend(
                [
                    f"Object {index}: {detection.get('class_name') or '--'} | "
                    f"Confidence: {confidence_text} | Object depth: {depth_text}",
                    f"Relative angle: {angle_text} | Object position: {position_text}",
                ]
            )
        if len(detections) > len(visible_detections):
            lines.append(f"+{len(detections) - len(visible_detections)} additional detection(s)")
        return "\n".join(lines)

    def _mission_ozet_metni(self, d):
        decision = d.get("mission_decision")
        if not isinstance(decision, dict):
            mission = str(d.get("active_mission") or "NO MISSION").upper()
            status, _ = self._vessel_status_bilgisi(d)
            if mission == "NO MISSION":
                return (
                    f"{mission} | {status}\n"
                    "ACTION: Waiting for mission selection\n"
                    "WHY: No task is currently active"
                )
            return (
                f"{mission} | {status}\n"
                "ACTION: Waiting for Jetson decision status\n"
                "WHY: Mission decision stream not received"
            )

        mission = str(decision.get("active_mission") or "MISSION").upper()
        if mission in ("TASK1", "TASK2", "TASK3", "TASK4"):
            mission = f"TASK {mission[-1]}"
        stage = str(decision.get("stage") or "ACTIVE").upper()
        current_target = int(decision.get("current_target", 0) or 0)
        target_count = int(decision.get("target_count", 0) or 0)
        progress = float(decision.get("progress_percent", 0.0) or 0.0)
        target_text = f" | WP {current_target}/{target_count}" if target_count else ""
        action = str(decision.get("action") or "Monitor mission")
        reason = str(decision.get("reason") or "Mission status update")

        valid_detections = []
        for detection in d.get("detections", []):
            if not isinstance(detection, dict):
                continue
            try:
                depth = float(detection.get("depth"))
            except (TypeError, ValueError):
                continue
            if math.isfinite(depth) and depth > 0.0:
                valid_detections.append((depth, detection))

        if valid_detections:
            depth, nearest = min(valid_detections, key=lambda item: item[0])
            class_name = str(nearest.get("class_name") or "Unknown object")
            class_name = class_name.replace("_", " ").capitalize()
            try:
                angle = float(nearest.get("angle"))
            except (TypeError, ValueError):
                angle = float("nan")
            if math.isfinite(angle):
                if angle > 0.5:
                    side = "starboard"
                elif angle < -0.5:
                    side = "port"
                else:
                    side = "ahead"
                angle_text = f"{angle:+.1f}° {side}"
            else:
                angle_text = "angle --"
            try:
                confidence = float(nearest.get("confidence")) * 100.0
            except (TypeError, ValueError):
                confidence = float("nan")
            confidence_text = f"{confidence:.0f}%" if math.isfinite(confidence) else "--"
            observation = (
                f"{class_name} | {depth:.2f} m | {angle_text} | {confidence_text}"
            )
        else:
            observation = "No active object detection"

        try:
            waypoint_distance = float(d.get("mesafe", 0.0) or 0.0)
        except (TypeError, ValueError):
            waypoint_distance = 0.0
        try:
            heading = float(d.get("yaw", 0.0) or 0.0)
        except (TypeError, ValueError):
            heading = 0.0
        armed = bool(d.get("armed"))
        disarm_requested = bool(d.get("arm_change_pending")) and not bool(
            d.get("requested_arm_state")
        )
        try:
            sog = float(d.get("hiz", 0.0) or 0.0) if armed and not disarm_requested else 0.0
        except (TypeError, ValueError):
            sog = 0.0
        return (
            f"{mission} | {stage}{target_text} | {progress:.0f}%\n"
            f"OBSERVATION: {observation}\n"
            f"WHY: {reason}\n"
            f"ACTION: {action}\n"
            f"STATUS: WP distance {waypoint_distance:.1f} m | "
            f"Heading {heading:.1f}° | SOG {sog:.1f} m/s"
        )

    def _decision_metni(self, d):
        colreg = d.get("active_colreg_rule") or d.get("colreg_rule") or d.get("colreg") or "--"
        situation = (
            d.get("colreg_situation")
            or d.get("situation")
            or d.get("encounter_type")
            or d.get("encounter")
            or "--"
        )
        decision = d.get("maneuver") or d.get("decision") or d.get("action") or "--"
        reason = d.get("decision_reason") or d.get("colreg_reason") or d.get("reason") or "--"
        collision_risk = d.get("collision_risk") or d.get("risk") or d.get("risk_level") or "--"
        target_distance = (
            d.get("target_vessel_distance")
            or d.get("target_distance")
            or d.get("obstacle_distance")
            or d.get("distance_to_target")
        )
        if target_distance is None:
            target_distance_text = "--"
        else:
            try:
                target_distance_text = f"{float(target_distance):.1f} m"
            except (TypeError, ValueError):
                target_distance_text = str(target_distance)

        return (
            f"Active COLREG rule: {colreg or '--'}\n\n"
            f"Situation: {situation or '--'}\n\n"
            f"Decision: {decision or '--'}\n\n"
            f"Reason: {reason or '--'}\n\n"
            f"Collision risk: {collision_risk or '--'}\n\n"
            f"Target vessel distance: {target_distance_text}"
        )

        mesafe = d.get("distance_to_next_wp", d.get("mesafe", 0.0))
        colreg = d.get("colreg_rule", d.get("colreg", "--"))
        decision = d.get("maneuver", d.get("decision", "--"))
        reason = d.get("decision_reason", d.get("colreg_reason", "--"))
        mission = d.get("active_mission") or "--"
        status, _renk = self._vessel_status_bilgisi(d)
        compact_status = status
        for prefix in ("🔵 ", "🟢 ", "🟡 ", "🔴 "):
            compact_status = compact_status.replace(prefix, "")
        compact_status = compact_status.split(" - ", 1)[0]
        return (
            f"Mission: {mission}\n\n"
            f"Status: {compact_status}\n\n"
            f"Distance to next WP: {float(mesafe or 0.0):.1f} m\n\n"
            f"COLREG rule: {colreg or '--'}\n\n"
            f"Decision: {decision or '--'}\n\n"
            f"Reason: {reason or '--'}"
        )

    def _vessel_status_bilgisi(self, d):
        mod = str(d.get("mod", "UNKNOWN") or "UNKNOWN").upper()
        baglanti = bool(d.get("baglanti"))
        link_ok = bool(d.get("link_ok"))
        heartbeat_seen = bool(d.get("heartbeat_seen"))
        telemetry_lost = bool(d.get("telemetry_lost"))
        armed = bool(d.get("armed"))
        mission_active = bool(d.get("active_mission"))

        if not baglanti:
            return "OUT OF CONTROL", "#c0392b"
        if telemetry_lost:
            return "OUT OF CONTROL", "#c0392b"
        if not heartbeat_seen or not link_ok:
            return "OUT OF CONTROL", "#c0392b"
        if not armed:
            return "STANDBY", "#2980b9"
        if mod in ("AUTO", "GUIDED"):
            return "AUTONOMOUS", "#27ae60"
        if mod in ("MANUAL", "HOLD", "STEERING", "LEARNING", "ACRO", "LOITER"):
            return "REMOTE CONTROL", "#f1c40f"
        if armed:
            return "REMOTE CONTROL", "#f1c40f"
        return "STANDBY", "#2980b9"

    def _vessel_status_guncelle(self, d):
        if self._vessel_status_label is None:
            return
        status, renk = self._vessel_status_bilgisi(d)
        hedef = (status, renk)
        simdi = time.time()

        if self._vessel_status_son == hedef:
            self._vessel_status_aday = None
            return

        onceki_status = self._vessel_status_son[0] if self._vessel_status_son else ""
        no_telemetryden_cikis = onceki_status == "OUT OF CONTROL" and status != onceki_status

        if self._vessel_status_son is not None and not no_telemetryden_cikis:
            if self._vessel_status_aday != hedef:
                self._vessel_status_aday = hedef
                self._vessel_status_aday_zamani = simdi
                return
            if simdi - self._vessel_status_aday_zamani < 1.0:
                return

        self._vessel_status_label.setText(status)
        self._vessel_status_label.setStyleSheet(self._vessel_status_stili(renk))
        self._vessel_status_son = hedef
        self._vessel_status_aday = None

    def _batarya_guncelle(self, d):
        battery = d.get("battery", {})
        voltage = float(d.get("voltaj", battery.get("total_voltage", 0.0)) or 0.0)
        # No current sensor is installed, so Pixhawk's remaining percentage can
        # remain near 100%. Estimate SOC consistently from the filtered 6S LiPo
        # pack voltage instead.
        percent = _pil_yuzdesi_voltajdan(voltage)
        try:
            capacity_wh = float(
                d.get(
                    "capacity_wh",
                    battery.get("capacity_wh", DEFAULT_BATTERY_CAPACITY_WH),
                )
            )
        except (TypeError, ValueError):
            capacity_wh = DEFAULT_BATTERY_CAPACITY_WH
        if capacity_wh <= 0:
            capacity_wh = DEFAULT_BATTERY_CAPACITY_WH
        remaining_wh = capacity_wh * percent / 100.0

        self.progressBar.setValue(percent)
        if hasattr(self, "lcdNumber_2"):
            self.lcdNumber_2.display(voltage)
        if hasattr(self, "LBATTERYWH"):
            self.LBATTERYWH.setText(f"ENERGY LEFT: {remaining_wh:.1f} Wh")

    def _arac_bagli_mi(self, d):
        return bool(
            d.get("baglanti")
            and d.get("link_ok")
            and d.get("heartbeat_seen")
            and not d.get("telemetry_lost")
        )

    def _arac_komut_butonlari(self):
        return (
            self.pushButton_4,
            self.pushButton_7,
            self.pushButton_6,
            self.comboBox_2,
        )

    def _arac_komutlarini_guncelle(self, bagli, durum=None):
        aktif = bool(bagli)
        if self._arac_komutlari_aktif == aktif:
            self._acil_stop_butonunu_guncelle(aktif, durum)
            return

        self._arac_komutlari_aktif = aktif
        for widget in self._arac_komut_butonlari():
            if widget.isEnabled() != aktif:
                widget.setEnabled(aktif)
        if not aktif:
            self._arac_komutlarini_kilitle()
        else:
            self._acil_stop_butonunu_guncelle(True, durum)

    def _acil_stop_butonunu_guncelle(self, bagli, durum=None):
        aktif = bool(bagli)
        durum = durum or {}
        relay_active = bool(durum.get("emergency_relay_active"))
        relay_pending = bool(durum.get("emergency_relay_pending"))
        gorunum = (
            aktif,
            relay_active,
            relay_pending,
            bool(getattr(self, "_kompakt_duzen_aktif", False)),
        )
        if self._stop_buton_gorunum_durumu == gorunum:
            return
        self._stop_buton_gorunum_durumu = gorunum

        buton_aktif = aktif and not relay_pending
        if self.pushButton_7.isEnabled() != buton_aktif:
            self.pushButton_7.setEnabled(buton_aktif)
        if not aktif:
            self.pushButton_7.setStyleSheet(self._acil_stop_stili)
            self.pushButton_7.setText("STOP LOCKED")
        elif relay_pending:
            self.pushButton_7.setStyleSheet(self._relay_pending_stili)
            self.pushButton_7.setText(
                "CUTTING POWER..." if not relay_active else "SAFE STARTING..."
            )
        elif relay_active:
            self.pushButton_7.setStyleSheet(self._safe_start_stili)
            self.pushButton_7.setText("SAFE START")
        else:
            self.pushButton_7.setStyleSheet(self._acil_stop_stili)
            self.pushButton_7.setText("EMERGENCY STOP")

    def _arac_komutlarini_kilitle(self):
        self._arm_komut_bekliyor = False
        self._arm_komut_hedef = None
        self._arm_durum_aday = None
        arm_durum = ("locked", bool(getattr(self, "_kompakt_duzen_aktif", False)))
        if self._arm_buton_gorunum_durumu != arm_durum:
            self._arm_buton_gorunum_durumu = arm_durum
            self.pushButton_4.setStyleSheet(self._komut_buton_stili)
            self.pushButton_4.setText("ARM LOCKED")
        if hasattr(self, "pushButton_8"):
            self.pushButton_8.hide()
        self._stop_buton_gorunum_durumu = None
        self._acil_stop_butonunu_guncelle(False, {})
        self.pushButton_6.setStyleSheet(self._komut_buton_stili)
        self.pushButton_6.setText("NO VEHICLE")
        self.comboBox_2.setStyleSheet(self._mode_combo_stili)
        self._mode_combo_son_stil = self._mode_combo_stili

    def _komut_engelli_mi(self):
        if self._arac_komutlari_aktif:
            return False
        self.sistem.log_sinyali.emit("ERROR: Vehicle is not connected. Command blocked by GUI.")
        return True

    def tazele(self, d):
        self.pushButton_10.setText(f"HEADING: {d.get('yaw', 0.0):.1f}°")
        self.pushButton_5.setText(f"ROLL: {d.get('roll', 0.0):.1f}°")
        self.pushButton_9.setText(f"PITCH: {d.get('pitch', 0.0):.1f}°")
        armed = bool(d.get("armed"))
        disarm_requested = bool(d.get("arm_change_pending")) and not bool(
            d.get("requested_arm_state")
        )
        motion_enabled = armed and not disarm_requested
        displayed_speed = float(d.get("hiz", 0.0) or 0.0) if motion_enabled else 0.0
        displayed_cog = float(d.get("cog", 0.0) or 0.0) if motion_enabled else 0.0
        if self._cog_label is not None:
            if displayed_speed >= HaritaCizimKatmani.COG_MIN_SPEED_M_S:
                self._cog_label.setText(f"COG: {displayed_cog:.1f}°")
            else:
                self._cog_label.setText("COG: 0.0°")

        self.lcdNumber_3.display(displayed_speed)
        self.pushButton_11.setText(str(d.get("lat", 0.0)))
        self.pushButton_12.setText(str(d.get("lon", 0.0)))
        if getattr(self, "_ana_harita_katmani", None) is not None:
            self._ana_harita_katmani.set_vehicle(
                {
                    "lat": d.get("lat", 0.0),
                    "lon": d.get("lon", 0.0),
                    "yaw": d.get("yaw", 0.0),
                    "cog": displayed_cog,
                    "speed": displayed_speed,
                }
            )
        if hasattr(self, "detection_graph"):
            self.detection_graph.set_detections(d.get("detections", []))
        if hasattr(self, "LMAPGPS"):
            self.LMAPGPS.setText(f"GPS: {d.get('gps', 0)}")
        if hasattr(self, "LMAPSATS"):
            self.LMAPSATS.setText(f"SATS: {d.get('gps_uydu', 0)}")
        if hasattr(self, "LMAPDIST"):
            self.LMAPDIST.setText(f"NEXT WP: {float(d.get('mesafe', 0.0) or 0.0):.1f} m")
        self._batarya_guncelle(d)
        self.textEdit.setPlainText(self._detection_metni(d))
        self._vessel_status_guncelle(d)
        self._system_overview_guncelle(d)
        self.label_8.setText(self._mission_ozet_metni(d))

        bagli = self._arac_bagli_mi(d)
        if bagli:
            self.pushButton.setStyleSheet(
                "background-color: #2ecc71; color: white; font-weight: bold; "
                "border-radius: 8px; border: 2px solid #27ae60; padding: 8px;"
            )
            self.pushButton.setText("CONNECTED")
        elif d.get("telemetry_lost"):
            self.pushButton.setStyleSheet(
                "background-color: #c0392b; color: white; font-weight: bold; "
                "border-radius: 8px; border: 2px solid #922b21; padding: 8px;"
            )
            self.pushButton.setText("LINK LOST")
        elif d.get("baglanti"):
            self.pushButton.setStyleSheet(
                "background-color: #f39c12; color: white; font-weight: bold; "
                "border-radius: 8px; border: 2px solid #d68910; padding: 8px;"
            )
            self.pushButton.setText("CONNECTING...")
        else:
            self.pushButton.setStyleSheet(
                "background-color: #e74c3c; color: white; font-weight: bold; "
                "border-radius: 8px; border: 2px solid #c0392b; padding: 8px;"
            )
            self.pushButton.setText("CONNECT")

        if hasattr(self, "pushButton_wifi"):
            if d.get("wifi_aktif"):
                self.pushButton_wifi.setStyleSheet(
                    "background-color: #2ecc71; color: white; "
                    "font-weight: bold; border-radius: 5px;"
                )
                self.pushButton_wifi.setText("WI-FI ACTIVE")
            else:
                self.pushButton_wifi.setStyleSheet(
                    "background-color: #e74c3c; color: white; "
                    "font-weight: bold; border-radius: 5px;"
                )
                self.pushButton_wifi.setText("WI-FI INACTIVE")

        self._arac_komutlarini_guncelle(bagli, d)
        if not bagli:
            return

        self._mode_combo_pixhawk_ile_esitle(d)
        self._arm_durumunu_isle(d)

        bekliyor_turuncu = (
            "background-color: #e67e22; color: white; font-weight: bold; "
            "border-radius: 12px; padding: 8px; border: 1px solid #d35400;"
        )
        onaylandi_mavi = (
            "background-color: #3498db; color: white; font-weight: bold; "
            "border-radius: 12px; padding: 8px; border: 1px solid #2980b9;"
        )

        if d.get("mode_change_pending") and d.get("requested_mode") == 10:
            self.pushButton_6.setStyleSheet(bekliyor_turuncu)
            self.pushButton_6.setText("AUTO TRANSITION PENDING...")
        elif d.get("mod_id") == 10:
            self.pushButton_6.setStyleSheet(onaylandi_mavi)
            self.pushButton_6.setText("AUTO ACTIVE")
        else:
            self.pushButton_6.setStyleSheet(self._komut_buton_stili)
            self.pushButton_6.setText("EXECUTE MISSION")

        self._mode_combo_durumunu_guncelle(d, bekliyor_turuncu, onaylandi_mavi)

    def arm_disarm_degistir(self):
        if self._komut_engelli_mi():
            return
        if self._arm_komut_bekliyor:
            self.sistem.log_sinyali.emit("INFO: ARM/DISARM command is waiting for vehicle confirmation.")
            return
        if self._son_armed_durum:
            self._arm_komut_beklemeye_al(False)
            self.sistem.disarm_yap()
            return
        if self.comboBox_2.findText("HOLD") >= 0:
            blocker = QtCore.QSignalBlocker(self.comboBox_2)
            self.comboBox_2.setCurrentText("HOLD")
            del blocker
        self._arm_komut_beklemeye_al(True)
        self.sistem.arm_yap()

    def arm_yap(self):
        self.arm_disarm_degistir()

    def disarm_yap(self):
        if self._komut_engelli_mi():
            return
        self.sistem.disarm_yap()

    def acil_durum(self):
        if self._komut_engelli_mi():
            return
        self.sistem.acil_durum()

    def mod_secildi(self, mod_adi):
        if not self._ui_hazir or not mod_adi or self._komut_engelli_mi():
            return
        durum = self.sistem.durum_al()
        dogrulanmis_mod = str(durum.get("mod", "") or "").upper()
        if dogrulanmis_mod and dogrulanmis_mod != "UNKNOWN":
            index = self.comboBox_2.findText(dogrulanmis_mod)
            if index < 0:
                self.comboBox_2.addItem(dogrulanmis_mod)
                index = self.comboBox_2.findText(dogrulanmis_mod)
            if index >= 0:
                blocker = QtCore.QSignalBlocker(self.comboBox_2)
                self.comboBox_2.setCurrentIndex(index)
                del blocker
        if durum.get("mode_change_pending"):
            self.sistem.log_sinyali.emit(
                "INFO: Mode selection is locked until Pixhawk confirms the current request."
            )
            return
        if str(mod_adi).upper() == dogrulanmis_mod:
            return
        self.comboBox_2.setEnabled(False)
        self.sistem.mod_ayarla_ad(mod_adi)

    def _mode_combo_pixhawk_ile_esitle(self, d):
        if d.get("mode_change_pending"):
            return

        mod = str(d.get("mod", "") or "").upper()
        if not mod or mod == "UNKNOWN":
            return

        simdi = time.time()
        if mod == self._mode_combo_son_pixhawk_mod:
            self._mode_combo_aday = None
            return

        if self._mode_combo_aday != mod:
            self._mode_combo_aday = mod
            self._mode_combo_aday_zamani = simdi
            return

        if simdi - self._mode_combo_aday_zamani < 1.0:
            return

        index = self.comboBox_2.findText(mod)
        if index < 0:
            self.comboBox_2.addItem(mod)
            index = self.comboBox_2.findText(mod)
        if index < 0:
            return
        if self.comboBox_2.currentIndex() == index:
            self._mode_combo_son_pixhawk_mod = mod
            self._mode_combo_aday = None
            return

        blocker = QtCore.QSignalBlocker(self.comboBox_2)
        self.comboBox_2.setCurrentIndex(index)
        del blocker
        self._mode_combo_son_pixhawk_mod = mod
        self._mode_combo_aday = None

    def _mode_combo_durumunu_guncelle(self, d, bekliyor_stil, aktif_stil):
        pending = bool(d.get("mode_change_pending"))
        if pending:
            yeni_stil = bekliyor_stil
        else:
            yeni_stil = self._mode_combo_stili

        combo_aktif = bool(self._arac_komutlari_aktif and not pending)
        if self.comboBox_2.isEnabled() != combo_aktif:
            self.comboBox_2.setEnabled(combo_aktif)

        if self._mode_combo_son_stil != yeni_stil:
            self.comboBox_2.setStyleSheet(yeni_stil)
            self._mode_combo_son_stil = yeni_stil

    def _arm_durumunu_isle(self, d):
        armed = bool(d.get("armed"))
        pending = bool(d.get("arm_change_pending"))
        simdi = time.time()

        if pending and not self._arm_komut_bekliyor:
            self._arm_komut_beklemeye_al(bool(d.get("requested_arm_state")))

        if self._arm_komut_bekliyor:
            hedef_onaylandi = armed == self._arm_komut_hedef and not pending
            zaman_asimi = simdi - self._arm_komut_baslangic > 8.0
            if hedef_onaylandi or zaman_asimi:
                if zaman_asimi and not hedef_onaylandi:
                    self.sistem.log_sinyali.emit("WARNING: ARM/DISARM confirmation timed out; button unlocked.")
                self._arm_komut_bekliyor = False
                self._arm_komut_hedef = None
                self._arm_durum_aday = None
                self._arm_toggle_butonunu_guncelle(armed)
            else:
                self._arm_buton_bekleme_gorunumu()
            return

        if armed == self._son_armed_durum:
            self._arm_durum_aday = None
            self._arm_toggle_butonunu_guncelle(armed)
            return

        if self._arm_durum_aday != armed:
            self._arm_durum_aday = armed
            self._arm_durum_aday_zamani = simdi
            return

        if simdi - self._arm_durum_aday_zamani >= 0.8:
            self._arm_durum_aday = None
            self._arm_toggle_butonunu_guncelle(armed)

    def _arm_komut_beklemeye_al(self, hedef):
        self._arm_komut_bekliyor = True
        self._arm_komut_hedef = bool(hedef)
        self._arm_komut_baslangic = time.time()
        self._arm_buton_bekleme_gorunumu()

    def _arm_buton_bekleme_gorunumu(self):
        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        metin = "ARMING..." if self._arm_komut_hedef else "DISARMING..."
        durum = ("pending", metin, kompakt)
        if self._arm_buton_gorunum_durumu == durum:
            return

        self._arm_buton_gorunum_durumu = durum
        if self.pushButton_4.isEnabled():
            self.pushButton_4.setEnabled(False)
        self.pushButton_4.setMinimumSize(88 if kompakt else 180, 36 if kompakt else 50)
        self.pushButton_4.setStyleSheet(
            "background-color: #34495e; color: white; "
            "font-weight: bold; border-radius: 8px; "
            "border: 2px solid #2c3e50; padding: 8px 18px;"
        )
        self.pushButton_4.setText(metin)

    def _arm_toggle_butonunu_guncelle(self, armed):
        self._son_armed_durum = bool(armed)
        kompakt = bool(getattr(self, "_kompakt_duzen_aktif", False))
        durum = ("armed" if armed else "disarmed", kompakt)
        if self._arm_buton_gorunum_durumu == durum:
            return

        self._arm_buton_gorunum_durumu = durum
        if not self.pushButton_4.isEnabled() and self._arac_komutlari_aktif:
            self.pushButton_4.setEnabled(True)
        self.pushButton_4.setMinimumSize(88 if kompakt else 180, 36 if kompakt else 50)
        if armed:
            self.pushButton_4.setStyleSheet(
                "background-color: #e74c3c; color: white; "
                "font-weight: bold; border-radius: 6px; "
                "border: 2px solid #c0392b; padding: 8px 18px;"
            )
            self.pushButton_4.setText("DISARM")
        else:
            self.pushButton_4.setStyleSheet(
                "background-color: #2ecc71; color: white; "
                "font-weight: bold; border-radius: 10px; "
                "border: 2px solid #27ae60; padding: 8px 18px;"
            )
            self.pushButton_4.setText("ARM")

    def _armed_butonunu_sabitle(self):
        self._arm_toggle_butonunu_guncelle(True)

    def _disarmed_butonunu_sabitle(self):
        self._arm_toggle_butonunu_guncelle(False)
        if hasattr(self, "pushButton_8"):
            self.pushButton_8.hide()

    def _arm_butonlarini_sabitle(self):
        self._arm_toggle_butonunu_guncelle(self._son_armed_durum)
        if hasattr(self, "pushButton_8"):
            self.pushButton_8.hide()

    def _log_dosyasini_hazirla(self):
        try:
            logs_klasoru = Path(PROJE_KLASORU) / "logs"
            logs_klasoru.mkdir(parents=True, exist_ok=True)
            log_yolu = logs_klasoru / f"pruva_gui_{datetime.now():%Y%m%d_%H%M%S}.log"
            log_yolu.write_text(
                f"{datetime.now():%Y-%m-%d %H:%M:%S} | PRUVA GUI SESSION STARTED\n",
                encoding="utf-8",
            )
            return log_yolu
        except OSError:
            return None

    def _log_dosyaya_yaz(self, mesaj):
        if self._log_dosyasi is None:
            return
        try:
            with self._log_dosyasi.open("a", encoding="utf-8") as log_file:
                log_file.write(f"{datetime.now():%Y-%m-%d %H:%M:%S.%f} | {mesaj}\n")
        except OSError:
            self._log_dosyasi = None

    def log_ekle(self, m):
        m = str(m)
        self._log_dosyaya_yaz(m)
        temel_log_stili = "font-size: 12pt; font-weight: bold;"
        if "!!!" in m or "ERROR" in m or "FAIL" in m:
            stil = temel_log_stili + " color: #e74c3c;"
        elif (
            "COMPLETED" in m
            or "SUCCESS" in m
            or "CONFIRMED" in m
        ):
            stil = temel_log_stili + " color: #2ecc71;"
        else:
            stil = temel_log_stili + " color: #3498db;"

        self._log_gecmisi.append((f">> {m}", stil))
        log_limit = 80 if hasattr(self, "TXTSTATUSLOG") else len(self._log_etiketleri)
        self._log_gecmisi = self._log_gecmisi[-log_limit:]

        if hasattr(self, "TXTSTATUSLOG"):
            html_lines = []
            for metin, satir_stili in self._log_gecmisi:
                color = "#3498db"
                if "#e74c3c" in satir_stili:
                    color = "#e74c3c"
                elif "#2ecc71" in satir_stili:
                    color = "#2ecc71"
                safe_text = (
                    metin.replace("&", "&amp;")
                    .replace("<", "&lt;")
                    .replace(">", "&gt;")
                )
                html_lines.append(
                    f"<div style='font-size:11pt;font-weight:700;color:{color};white-space:nowrap;'>{safe_text}</div>"
                )
            self.TXTSTATUSLOG.setHtml("".join(html_lines))
            scrollbar = self.TXTSTATUSLOG.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
            return

        for i, etiket in enumerate(self._log_etiketleri):
            if i < len(self._log_gecmisi):
                metin, satir_stili = self._log_gecmisi[i]
                etiket.setText(metin)
                etiket.setStyleSheet(satir_stili)
            else:
                etiket.setText("")
                etiket.setStyleSheet("")

    def icra(self):
        if self._komut_engelli_mi():
            return
        if self.radioButton.isChecked():
            gorev = "M1"
        elif self.radioButton_2.isChecked():
            gorev = "M2"
        elif self.radioButton_3.isChecked():
            gorev = "M3"
        elif self.radioButton_4.isChecked():
            gorev = "M4"
        else:
            self.sistem.log_sinyali.emit("ERROR: Select a mission before EXECUTE. Vehicle remains in HOLD.")
            self.sistem.mod_ayarla_ad("HOLD")
            return
        self.sistem.gorev_baslat(gorev)

    def closeEvent(self, event):
        self.sistem.kapat()
        event.accept()
