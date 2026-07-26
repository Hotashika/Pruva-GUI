import copy
import ipaddress
import math
import os
import platform
import re
import socket
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from threading import Lock, Thread

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage

# MAVLink TUNNEL has message id 385 and is only available in MAVLink 2.
# This must be set before importing pymavlink's generated dialect.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

from .decision_protocol import DECISION_PAYLOAD_TYPE, decode_decision_payload
from .detection_protocol import DETECTION_PAYLOAD_TYPE, decode_detection_payload
from ..services.backend_client import BackendClient
from ..streaming import video_client as grap_video


ARDUROVER_MODS = {
    0: "MANUAL",
    1: "ACRO",
    2: "LEARNING",
    3: "STEERING",
    4: "HOLD",
    5: "LOITER",
    6: "FOLLOW",
    7: "SIMPLE",
    8: "DOCK",
    9: "CIRCLE",
    10: "AUTO",
    11: "RTL",
    12: "SMART_RTL",
    15: "GUIDED",
    16: "INITIALISING",
}
MODE_NAME_TO_ID = {name: mode_id for mode_id, name in ARDUROVER_MODS.items()}

HEARTBEAT_TIMEOUT = 5.0
MODE_CHANGE_TIMEOUT = 8.0
RELAY_COMMAND_TIMEOUT = 5.0
# Pixhawk configuration:
# RELAY5_PIN=54, RELAY5_DEFAULT=0, RELAY5_INVERTED=1,
# SERVO13_FUNCTION=-1. ArduPilot MAVLink relay numbering is zero-based, so
# RELAY5 is Relay No 4 and its pin 54 is physical AUX OUT 5. Because the output
# is inverted, logical OFF produces physical HIGH and logical ON produces LOW.
EMERGENCY_RELAY_NUMBER = int(os.getenv("EMERGENCY_RELAY_NUMBER", "4"))
EMERGENCY_RELAY_INVERTED = (
    os.getenv("EMERGENCY_RELAY_INVERTED", "1").strip().lower()
    in {"1", "true", "yes", "on"}
)
EMERGENCY_RELAY_DEFAULT_ON = (
    os.getenv("EMERGENCY_RELAY_DEFAULT_ON", "0").strip().lower()
    in {"1", "true", "yes", "on"}
)
# Hardware test: logical relay ON cuts power and logical OFF enables power.
EMERGENCY_RELAY_INITIAL_ACTIVE = EMERGENCY_RELAY_DEFAULT_ON
ARDUPILOT_FORCE_ARM_MAGIC = 21196
BATTERY_EMPTY_VOLTAGE = 21.0
BATTERY_FULL_VOLTAGE = 25.2
# 6S 10 Ah battery: 22.2 V nominal x 10 Ah = 222 Wh.
BATTERY_CAPACITY_WH = 222.0
BATTERY_VOLTAGE_FILTER_ALPHA = 0.20
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

JETSON_IP = os.getenv("JETSON_IP", "10.149.150.143").strip() or None
JETSON_MAC = "8c:b8:7e:04:20:a9"
JETSON_VIDEO_PORT = 5000
JETSON_BACKEND_PORT = 8000
NETWORK_SCAN_INTERVAL = 30.0
DETECTION_STALE_SEC = 1.0
DETECTION_FRAME_LIMIT = 8
DECISION_STALE_SEC = 3.0
MAX_VALID_SOG_M_S = float(os.getenv("NJORD_MAX_VALID_SOG_M_S", "12.0"))
SOG_FILTER_ALPHA = 0.35


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




class NjordVeriSistemi(QObject):
    veri_guncelle = pyqtSignal(dict)
    log_sinyali = pyqtSignal(str)
    kamera_sinyali = pyqtSignal(object)
    baglanti_kesildi = pyqtSignal()
    waypoint_guncelle = pyqtSignal(list)

    def __init__(self):
        super().__init__()

        self.connection = None
        self._lock = Lock()
        self._aktif = True
        self._wifi_monitor_running = True
        self._last_hb = 0.0
        self._connection_started_at = 0.0
        self._heartbeat_seen = False
        self._vehicle_system_id = None
        self._vehicle_component_id = None
        self._ignored_heartbeat_sources = set()
        self._telemetry_lost_reported = False
        self._last_read_error_log = 0.0
        self._watchdog_started = False
        self._streams_requested = False
        self._seen_message_types = set()
        self._mission_messages = []
        self._filtered_cog = None
        self._filtered_sog = 0.0
        self._filtered_battery_voltage = None
        self._last_sog_reject_log = 0.0
        self._last_radio_failsafe = 0.0

        self._camera_started = False
        self._camera_running = False
        self._last_video_frame_time = 0.0
        self._last_network_scan = 0.0
        self._last_logged_jetson_ip = None
        self._last_video_wait_log = 0.0
        self._last_mode_failure_diagnostic = 0.0
        self._mode_change_requested_at = 0.0
        self._emergency_relay_requested_at = 0.0
        self._emergency_relay_requested_state = None
        self.backend_client = BackendClient()
        self._mission_id = None
        self._mission_uploaded_to_pixhawk = False
        self._mission_component_zero = False
        self._mission_waypoints = []
        self._connection_busy = False

        self._durum = {
            "baglanti": False,
            "armed": False,
            "mod": "UNKNOWN",
            "mod_id": -1,
            "system_status": "UNKNOWN",
            "hiz": 0.0,
            "yaw": 0.0,
            "cog": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "gps": 0,
            "gps_uydu": 0,
            "mesafe": 0.0,
            "decision_log": "System ready. Waiting for connection...",
            "detection": None,
            "detections": [],
            "mission_decision": None,
            "active_mission": None,
            "link_ok": False,
            "heartbeat_seen": False,
            "telemetry_lost": False,
            "radio_failsafe": False,
            "arm_change_pending": False,
            "requested_arm_state": False,
            "mode_change_pending": False,
            "requested_mode": -1,
            "emergency_relay_active": EMERGENCY_RELAY_INITIAL_ACTIVE,
            "emergency_relay_pending": False,
            "wifi_aktif": False,
            "jetson_ip": "Searching...",
            "battery": {
                "total_voltage": 0.0,
                "percentage": 0,
                "remaining_wh": 0.0,
                "capacity_wh": BATTERY_CAPACITY_WH,
            },
        }

        Thread(target=self._wifi_kontrol_dongusu, daemon=True, name="WiFiWatch").start()

    def _set(self, **kwargs):
        with self._lock:
            self._durum.update(kwargs)
            armed = bool(self._durum.get("armed"))
            disarm_requested = bool(self._durum.get("arm_change_pending")) and not bool(
                self._durum.get("requested_arm_state")
            )
            if not armed or disarm_requested:
                # Central invariant: no telemetry source may leave motion data
                # non-zero while the vehicle is disarmed or disarming.
                self._durum["hiz"] = 0.0
                self._durum["cog"] = 0.0
                self._filtered_cog = None
                self._filtered_sog = 0.0
        self._emit_durum()

    def _snapshot(self):
        with self._lock:
            return copy.deepcopy(self._durum)

    def _telemetri_saglikli_mi(self):
        with self._lock:
            baglanti = bool(self._durum.get("baglanti"))
            heartbeat_seen = bool(self._durum.get("heartbeat_seen"))
            telemetry_lost = bool(self._durum.get("telemetry_lost"))
            son_hb = self._last_hb

        if not baglanti or not heartbeat_seen or telemetry_lost or son_hb <= 0:
            return False
        return time.time() - son_hb <= HEARTBEAT_TIMEOUT

    def _telemetri_degerlerini_sifirla(self):
        self._filtered_cog = None
        self._filtered_sog = 0.0
        return {
            "hiz": 0.0,
            "yaw": 0.0,
            "cog": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "gps": 0,
            "gps_uydu": 0,
            "mesafe": 0.0,
            "detection": None,
            "detections": [],
            "mission_decision": None,
        }

    def _telemetri_kaybi_isle(self, decision_log, physical_disconnect=False):
        self._mission_uploaded_to_pixhawk = False
        self._telemetry_lost_reported = True
        self._mode_change_requested_at = 0.0
        self._emergency_relay_requested_at = 0.0
        self._emergency_relay_requested_state = None
        self._set(
            **self._telemetri_degerlerini_sifirla(),
            baglanti=not physical_disconnect,
            link_ok=False,
            heartbeat_seen=False if physical_disconnect else self._heartbeat_seen,
            telemetry_lost=not physical_disconnect,
            armed=False,
            mod="UNKNOWN",
            mod_id=-1,
            active_mission=None,
            arm_change_pending=False,
            mode_change_pending=False,
            requested_mode=-1,
            emergency_relay_pending=False,
            decision_log=decision_log,
        )

    def _battery_state(self):
        battery = self._durum.get("battery")
        if not isinstance(battery, dict):
            battery = {}
            self._durum["battery"] = battery

        defaults = {
            "total_voltage": 0.0,
            "percentage": 0,
            "remaining_wh": 0.0,
            "capacity_wh": BATTERY_CAPACITY_WH,
        }
        for key, value in defaults.items():
            battery.setdefault(key, value)
        return battery

    def gorev_noktalarini_al(self):
        with self._lock:
            return copy.deepcopy(self._mission_waypoints)

    def gorev_noktalarini_guncelle(self, waypoints):
        normalized = []
        for index, item in enumerate(waypoints, start=1):
            if isinstance(item, dict):
                name = item.get("name") or item.get("id") or item.get("label") or f"WP_{index:02d}"
                lat = item.get("lat", item.get("latitude"))
                lon = item.get("lon", item.get("lng", item.get("longitude")))
            elif isinstance(item, (list, tuple)) and len(item) >= 3:
                name, lat, lon = item[0], item[1], item[2]
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                name, lat, lon = f"WP_{index:02d}", item[0], item[1]
            else:
                continue

            try:
                normalized.append(
                    {
                        "name": str(name),
                        "lat": float(lat),
                        "lon": float(lon),
                    }
                )
            except (TypeError, ValueError):
                continue

        with self._lock:
            self._mission_waypoints = normalized
        self.waypoint_guncelle.emit(copy.deepcopy(normalized))

    def _emit_durum(self):
        with self._lock:
            self._battery_state()
            now = time.monotonic()
            detections = [
                item for item in self._durum.get("detections", [])
                if now - float(item.get("received_monotonic", 0.0)) <= DETECTION_STALE_SEC
            ]
            self._durum["detections"] = detections
            self._durum["detection"] = (
                max(detections, key=lambda item: float(item.get("confidence", 0.0)))
                if detections
                else None
            )
            decision = self._durum.get("mission_decision")
            if (
                    isinstance(decision, dict)
                    and now - float(decision.get("received_monotonic", 0.0)) > DECISION_STALE_SEC
            ):
                self._durum["mission_decision"] = None
            kopya = copy.deepcopy(self._durum)

        battery = kopya.get("battery") or {}
        kopya["voltaj"] = battery.get("total_voltage", 0.0)
        kopya["pil_yuzde"] = battery.get("percentage", 0)
        kopya["remaining_wh"] = battery.get("remaining_wh", 0.0)
        kopya["capacity_wh"] = battery.get("capacity_wh", 0.0)
        self.veri_guncelle.emit(kopya)

    def _log(self, mesaj):
        print(f"[NJORD] {mesaj}")
        self.log_sinyali.emit(mesaj)

    def _arkaplan_calistir(self, ad, hedef, *args):
        def calistir():
            try:
                hedef(*args)
            except Exception as exc:
                self._log(f"ERROR: {ad} failed: {exc}")

        Thread(target=calistir, daemon=True, name=ad).start()

    def _command_output(self, command):
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                creationflags=self._subprocess_flags(),
            )
        except Exception:
            return None

        if platform.system().lower() == "windows":
            return result.stdout.decode("cp857", errors="replace")

        return result.stdout.decode(errors="replace")

    def get_ip_from_mac(self, target_mac):
        return self._arp_cache_ip(target_mac)

    @staticmethod
    def _ipv4_from_text(text):
        return re.findall(r"\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b", text or "")

    @staticmethod
    def _normalize_mac(value):
        return str(value or "").lower().replace("-", ":")

    def _arp_cache_ip(self, target_mac):
        normalized_mac = self._normalize_mac(target_mac)

        outputs = []
        for command in (["arp", "-a"], ["ip", "neigh"], ["ip", "neighbor"], ["netsh", "interface", "ip", "show", "neighbors"]):
            result = self._command_output(command)
            if result:
                outputs.append(result)

        for output in outputs:
            for line in output.splitlines():
                normalized_line = self._normalize_mac(line)
                if normalized_mac in normalized_line:
                    ips = self._ipv4_from_text(line)
                    if ips:
                        return ips[0]
        return None

    def _arp_ip_mac_eslesiyor(self, target_mac, ip):
        normalized_mac = self._normalize_mac(target_mac)
        ip = str(ip or "").strip()
        if not ip:
            return False

        for command in (["arp", "-a"], ["ip", "neigh"], ["ip", "neighbor"], ["netsh", "interface", "ip", "show", "neighbors"]):
            result = self._command_output(command)
            if not result:
                continue
            for line in result.splitlines():
                normalized_line = self._normalize_mac(line)
                if ip in line and normalized_mac in normalized_line:
                    return True
        return False

    def _local_ipv4_networks(self):
        system = platform.system().lower()
        if system == "windows":
            return self._local_ipv4_networks_windows()
        return self._local_ipv4_networks_posix()

    def _local_ipv4_networks_windows(self):
        result = self._command_output(["ipconfig"])
        networks = []
        current_ip = None
        for line in (result or "").splitlines():
            ipv4_match = re.search(r"IPv4.*?:\s*([0-9.]+)", line, re.IGNORECASE)
            mask_match = None
            if "mask" in line.lower() or "maske" in line.lower():
                mask_match = re.search(r":\s*([0-9.]+)", line)

            if ipv4_match:
                current_ip = ipv4_match.group(1)
            elif current_ip and mask_match:
                try:
                    network = ipaddress.IPv4Network(
                        f"{current_ip}/{mask_match.group(1)}",
                        strict=False,
                    )
                    if not network.is_loopback:
                        networks.append(network)
                except Exception:
                    pass
                current_ip = None

        if current_ip:
            try:
                networks.append(ipaddress.IPv4Network(f"{current_ip}/24", strict=False))
            except Exception:
                pass

        return networks

    def _local_ipv4_networks_posix(self):
        networks = []
        ip_output = self._command_output(["ip", "-4", "addr", "show"])
        if ip_output:
            for match in re.finditer(r"\binet\s+([0-9.]+/\d+)", ip_output):
                try:
                    network = ipaddress.IPv4Network(match.group(1), strict=False)
                    if not network.is_loopback:
                        networks.append(network)
                except Exception:
                    pass

        if not networks:
            ifconfig_output = self._command_output(["ifconfig"])
            for match in re.finditer(
                r"inet\s+(?:addr:)?([0-9.]+).*?(?:netmask\s+(0x[0-9a-fA-F]+|[0-9.]+))?",
                ifconfig_output or "",
            ):
                ip = match.group(1)
                mask = match.group(2) or "255.255.255.0"
                if mask.startswith("0x"):
                    mask_int = int(mask, 16)
                    mask = ".".join(str((mask_int >> shift) & 0xFF) for shift in (24, 16, 8, 0))
                try:
                    network = ipaddress.IPv4Network(f"{ip}/{mask}", strict=False)
                    if not network.is_loopback:
                        networks.append(network)
                except Exception:
                    pass

        unique = []
        seen = set()
        for network in networks:
            if network not in seen:
                unique.append(network)
                seen.add(network)
        return unique

    def _scan_network_for_mac(self, target_mac):
        networks = self._local_ipv4_networks()
        if not networks:
            return None

        for network in networks:
            hosts = [str(ip) for ip in network.hosts()]
            if len(hosts) > 4096:
                continue

            self._log(f"Searching Jetson MAC on network: {network}")
            with ThreadPoolExecutor(max_workers=64) as executor:
                futures = [executor.submit(self._ping_ip, ip) for ip in hosts]
                for future in as_completed(futures):
                    future.result()

            ip = self._arp_cache_ip(target_mac)
            if ip:
                self._log(f"Jetson MAC matched IP: {ip}")
                return ip

            self._log(f"Jetson MAC not found on network: {network}")

        return None

    def _find_video_stream_ip(self):
        networks = self._local_ipv4_networks()
        if not networks:
            return None

        for network in networks:
            hosts = [str(ip) for ip in network.hosts()]
            if len(hosts) > 4096:
                self._log(f"Skipping large network for video scan: {network}")
                continue

            self._log(f"Searching Jetson video service on {network}")
            with ThreadPoolExecutor(max_workers=64) as executor:
                futures = {
                    executor.submit(self._is_port_open, ip, JETSON_VIDEO_PORT): ip
                    for ip in hosts
                }
                for future in as_completed(futures):
                    ip = futures[future]
                    try:
                        if future.result():
                            self._log(f"Jetson video service found at IP: {ip}")
                            return ip
                    except Exception:
                        pass

            self._log(f"Jetson video service not found on {network}")

        return None

    def _is_port_open(self, ip, port):
        try:
            with socket.create_connection((ip, port), timeout=0.25):
                return True
        except OSError:
            return False

    def _jetson_reachable(self, ip):
        # An open Jetson service is stronger evidence than the ARP cache. Windows
        # may omit or age out the MAC entry while an existing video stream keeps
        # working, which previously made the UI report WI-FI LOST incorrectly.
        if not self._valid_ip(ip):
            return False
        backend_port = int(self.backend_client.config.get("backend", {}).get("http_port", JETSON_BACKEND_PORT))
        return self._is_port_open(ip, JETSON_VIDEO_PORT) or self._is_port_open(ip, backend_port)

    def _kamera_karesini_yayinla(self, image):
        if image is not None:
            now = time.monotonic()
            with self._lock:
                self._last_video_frame_time = now
                changed = not bool(self._durum.get("wifi_aktif"))
                self._durum["wifi_aktif"] = True
                if not self._valid_ip(self._durum.get("jetson_ip")) and JETSON_IP:
                    self._durum["jetson_ip"] = JETSON_IP
            if changed:
                self._emit_durum()
        self.kamera_sinyali.emit(image)

    def _valid_ip(self, value):
        return bool(re.fullmatch(r"(?:[0-9]{1,3}\.){3}[0-9]{1,3}", str(value)))

    def _subprocess_flags(self):
        if platform.system().lower() == "windows":
            return getattr(subprocess, "CREATE_NO_WINDOW", 0)
        return 0

    def _ping_ip(self, ip):
        if not ip:
            return False

        if platform.system().lower() == "windows":
            command = ["ping", "-n", "1", "-w", "1000", ip]
        else:
            command = ["ping", "-c", "1", "-W", "1", ip]

        try:
            result = subprocess.run(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                creationflags=self._subprocess_flags(),
            )
            return result.returncode == 0
        except Exception:
            return False

    def _wifi_kontrol_dongusu(self):
        # Jetson/Wi-Fi monitoring is independent from the Pixhawk connection.
        # A telemetry disconnect must not permanently stop network discovery.
        while self._wifi_monitor_running:
            bulunan_ip = None

            with self._lock:
                video_recent = (
                    self._last_video_frame_time > 0.0
                    and time.monotonic() - self._last_video_frame_time <= 3.0
                )
                current_ip = self._durum.get("jetson_ip")
            if video_recent:
                self._set(
                    wifi_aktif=True,
                    jetson_ip=current_ip if self._valid_ip(current_ip) else (JETSON_IP or "Connected"),
                )
                time.sleep(2.0)
                continue

            if JETSON_IP and self._jetson_reachable(JETSON_IP):
                self._set(wifi_aktif=True, jetson_ip=JETSON_IP)
            else:
                bulunan_ip = self.get_ip_from_mac(JETSON_MAC)
                if bulunan_ip and bulunan_ip != self._last_logged_jetson_ip:
                    self._last_logged_jetson_ip = bulunan_ip
                    self._log(f"Jetson found in ARP cache: {bulunan_ip}")
                if not bulunan_ip and time.time() - self._last_network_scan > NETWORK_SCAN_INTERVAL:
                    self._last_network_scan = time.time()
                    bulunan_ip = self._scan_network_for_mac(JETSON_MAC)

                if bulunan_ip and self._jetson_reachable(bulunan_ip):
                    self._set(wifi_aktif=True, jetson_ip=bulunan_ip)
                else:
                    self._set(wifi_aktif=False, jetson_ip=bulunan_ip or "Not found")

            time.sleep(2.0)

    def baglanti_kur(self, tip, baud, port):
        with self._lock:
            if self._connection_busy:
                self._log("INFO: Connection attempt is already running.")
                return
            self._connection_busy = True

        self._arkaplan_calistir("MAVLinkConnect", self._baglanti_kur_mavlink, tip, baud, port)

    def _baglanti_kur_mavlink(self, tip, baud, port):
        self._aktif = True
        self._log(f"CONNECTION STARTING: {tip} -> {port}")

        try:
            if tip == "UDP":
                address = f"udp:{port}" if ":" in port else f"udp:127.0.0.1:{port}"
                self.connection = mavutil.mavlink_connection(address)
            elif tip == "TCP":
                self.connection = mavutil.mavlink_connection(f"tcp:{port}")
            else:
                self.connection = mavutil.mavlink_connection(port, baud=int(baud))

            self._log("Waiting for Pixhawk heartbeat...")
            self._last_hb = 0.0
            self._connection_started_at = time.time()
            self._heartbeat_seen = False
            self._vehicle_system_id = None
            self._vehicle_component_id = None
            self._ignored_heartbeat_sources.clear()
            self._telemetry_lost_reported = False
            self._streams_requested = False
            self._mission_uploaded_to_pixhawk = False

            Thread(target=self._dinleme_dongusu, daemon=True, name="MAVLink").start()
            if not self._watchdog_started:
                self._watchdog_started = True
                Thread(target=self._guvenlik_dongusu, daemon=True, name="Watchdog").start()

            self._set(
                **self._telemetri_degerlerini_sifirla(),
                baglanti=True,
                link_ok=False,
                heartbeat_seen=False,
                telemetry_lost=False,
                active_mission=None,
                arm_change_pending=False,
                mode_change_pending=False,
                requested_mode=-1,
                emergency_relay_pending=False,
                decision_log="Connection established. Waiting for heartbeat...",
            )
            self._log("Telemetry connection started.")
            self._kamera_baslat()

        except Exception as exc:
            self._mission_uploaded_to_pixhawk = False
            self._log(f"CONNECTION ERROR: {exc}")
            self._set(
                **self._telemetri_degerlerini_sifirla(),
                baglanti=False,
                link_ok=False,
                heartbeat_seen=False,
                telemetry_lost=True,
                armed=False,
                active_mission=None,
                arm_change_pending=False,
                mode_change_pending=False,
                requested_mode=-1,
                emergency_relay_pending=False,
                decision_log=f"CONNECTION ERROR: {exc}",
            )
        finally:
            with self._lock:
                self._connection_busy = False

    def baglanti_kes(self):
        self._aktif = False
        self._camera_running = False
        self._camera_started = False
        self._watchdog_started = False
        self._mission_uploaded_to_pixhawk = False
        self._emergency_relay_requested_at = 0.0
        self._emergency_relay_requested_state = None

        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
        self.connection = None
        self._vehicle_system_id = None
        self._vehicle_component_id = None
        self._ignored_heartbeat_sources.clear()

        self._set(
            **self._telemetri_degerlerini_sifirla(),
            baglanti=False,
            link_ok=False,
            heartbeat_seen=False,
            telemetry_lost=False,
            mod="UNKNOWN",
            mod_id=-1,
            armed=False,
            active_mission=None,
            arm_change_pending=False,
            mode_change_pending=False,
            requested_mode=-1,
            emergency_relay_pending=False,
            decision_log="CONNECTION CLOSED",
        )
        self._log("CONNECTION CLOSED")

    def kapat(self):
        self._wifi_monitor_running = False
        self.baglanti_kes()

    def kamera_oto_baslat(self):
        self._kamera_baslat()

    def _kamera_baslat(self):
        if self._camera_started:
            return

        self._camera_started = True
        self._camera_running = True
        Thread(target=self._kamera_dongusu, daemon=True, name="Camera").start()

    def _kamera_dongusu(self):
        if grap_video is None:
            self._log("CAMERA WARNING: grap_video was not found. ZED2 stream could not start.")
            self._set(decision_log="Camera could not start: grap_video was not found.")
            self.kamera_sinyali.emit(None)
            self._camera_started = False
            self._camera_running = False
            return

        try:
            self._log("Camera waiting for Jetson IP...")
            self.kamera_sinyali.emit(None)
            jetson_ip = None
            while self._camera_running:
                with self._lock:
                    candidate_ip = self._durum.get("jetson_ip")
                    wifi_active = self._durum.get("wifi_aktif")

                if wifi_active and self._valid_ip(candidate_ip):
                    jetson_ip = candidate_ip
                    break

                time.sleep(1.0)

            if not jetson_ip:
                self._set(decision_log="Camera stopped before Jetson IP was found.")
                self.kamera_sinyali.emit(None)
                return

            while self._camera_running and not self._is_port_open(jetson_ip, JETSON_VIDEO_PORT):
                now = time.time()
                if now - self._last_video_wait_log > 5.0:
                    self._last_video_wait_log = now
                    self._log(
                        f"Waiting for Jetson video service: "
                        f"http://{jetson_ip}:{JETSON_VIDEO_PORT}/video_feed"
                    )
                    self._set(
                        decision_log=(
                            f"Jetson IP found ({jetson_ip}), "
                            f"waiting for video service on port {JETSON_VIDEO_PORT}..."
                        )
                    )
                time.sleep(1.0)

            if not self._camera_running:
                self._set(decision_log="Camera stopped before video service opened.")
                self.kamera_sinyali.emit(None)
                return

            self._set(decision_log=f"Camera connecting to Jetson IP: {jetson_ip}")

            try:
                grap_video.start(
                    jetson_ip=jetson_ip,
                    frame_callback=self._kamera_karesini_yayinla,
                    log_callback=self._log,
                    stop_callback=lambda: self._camera_running,
                )
            except TypeError:
                grap_video.start(
                    frame_callback=self._kamera_karesini_yayinla,
                    log_callback=self._log,
                    stop_callback=lambda: self._camera_running,
                )

        except Exception as exc:
            self._log(f"CAMERA THREAD ERROR: {exc}")
            self._set(decision_log=f"Camera error: {exc}")
            self.kamera_sinyali.emit(None)

        finally:
            self._camera_running = False
            self._camera_started = False
            self.kamera_sinyali.emit(None)
            self._log("CAMERA OFFLINE")

    def _dinleme_dongusu(self):
        while self._aktif and self.connection:
            try:
                msg = self.connection.recv_match(blocking=True, timeout=0.05)
                if msg:
                    self._islenmis_mesaj(msg)
            except Exception as exc:
                now = time.time()
                if now - self._last_read_error_log > 2.0:
                    self._last_read_error_log = now
                    self._log(f"WARNING: MAVLink read error, telemetry link lost: {exc}")
                if not self._telemetry_lost_reported:
                    self._telemetri_kaybi_isle(
                        "TELEMETRY DISCONNECTED! MAVLink read failed. Check telemetry cable/radio.",
                        physical_disconnect=True,
                    )
                try:
                    self.connection.close()
                except Exception:
                    pass
                self.connection = None
                break
                time.sleep(0.1)

    def _guvenlik_dongusu(self):
        while self._aktif:
            time.sleep(0.25)

            with self._lock:
                gecen_sure = time.time() - self._last_hb
                baglanti_var = self._durum["baglanti"]
                heartbeat_seen = self._heartbeat_seen
                baslangictan_beri = time.time() - self._connection_started_at if self._connection_started_at else 0.0
                telemetry_lost_reported = self._telemetry_lost_reported
                radio_failsafe = bool(self._durum.get("radio_failsafe"))
                mode_change_pending = bool(self._durum.get("mode_change_pending"))
                requested_mode = int(self._durum.get("requested_mode", -1))
                relay_pending = bool(self._durum.get("emergency_relay_pending"))

            if (
                mode_change_pending
                and self._mode_change_requested_at > 0
                and time.monotonic() - self._mode_change_requested_at > MODE_CHANGE_TIMEOUT
            ):
                requested_name = ARDUROVER_MODS.get(requested_mode, f"MODE_{requested_mode}")
                self._mode_change_requested_at = 0.0
                self._set(
                    mode_change_pending=False,
                    requested_mode=-1,
                    decision_log=(
                        f"{requested_name} mode was not confirmed by Pixhawk within "
                        f"{MODE_CHANGE_TIMEOUT:.0f} seconds. Current mode remains unchanged."
                    ),
                )
                self._log(
                    f"WARNING: {requested_name} MODE confirmation timed out; "
                    "using the latest heartbeat mode."
                )

            if (
                relay_pending
                and self._emergency_relay_requested_at > 0
                and time.monotonic() - self._emergency_relay_requested_at
                > RELAY_COMMAND_TIMEOUT
            ):
                self._emergency_relay_requested_at = 0.0
                self._emergency_relay_requested_state = None
                self._set(
                    emergency_relay_pending=False,
                    decision_log=(
                        "Emergency relay command was not confirmed by Pixhawk. "
                        "Relay state was not changed."
                    ),
                )
                self._log("ERROR: Emergency relay confirmation timed out.")

            if baglanti_var and not heartbeat_seen and baslangictan_beri > HEARTBEAT_TIMEOUT:
                if not telemetry_lost_reported:
                    self._log(
                        "ERROR: No Pixhawk heartbeat received. Check COM port, baud rate, telemetry radio power, and close Mission Planner."
                    )
                    self._heartbeat_seen = False
                    self._telemetri_kaybi_isle("No Pixhawk heartbeat. Check COM/baud and telemetry radio.")
            elif baglanti_var and heartbeat_seen and gecen_sure > HEARTBEAT_TIMEOUT:
                if not telemetry_lost_reported:
                    self._log("!!! WARNING: HEARTBEAT LOST !!!")
                    self._telemetri_kaybi_isle(f"TELEMETRY LOST! No heartbeat for {HEARTBEAT_TIMEOUT:.0f}s.")
            elif self.connection and not baglanti_var and gecen_sure <= HEARTBEAT_TIMEOUT and self._last_hb > 0:
                self._log("Heartbeat restored. Connection is stable.")
                self._set(baglanti=True, link_ok=True, heartbeat_seen=True, telemetry_lost=False)
            elif radio_failsafe and self._last_radio_failsafe and time.time() - self._last_radio_failsafe > 12.0:
                self._set(
                    radio_failsafe=False,
                    system_status="MAV_STATE_ACTIVE",
                    decision_log="Radio failsafe message cleared. Vehicle commands can be retried.",
                )

        self._watchdog_started = False

    def _islenmis_mesaj(self, msg):
        msg_type = msg.get_type()
        if msg_type != "BAD_DATA" and msg_type not in self._seen_message_types:
            self._seen_message_types.add(msg_type)
            if len(self._seen_message_types) <= 20:
                self._log(f"MAVLink message received: {msg_type}")

        if msg_type == "HEARTBEAT":
            if not self._arac_heartbeat_kaynagini_kabul_et(msg):
                return

            self._last_hb = time.time()
            self._heartbeat_seen = True
            self._telemetry_lost_reported = False
            if not self._streams_requested:
                self._mavlink_streamlerini_iste()

            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mod_id = int(msg.custom_mode)
            mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")
            system_status = self._mavlink_state_name(getattr(msg, "system_status", None))

            with self._lock:
                if self._durum.get("arm_change_pending"):
                    if self._durum.get("requested_arm_state") == armed:
                        self._durum["arm_change_pending"] = False
                        durum_str = "ARMED (ACTIVE)" if armed else "DISARMED (LOCKED)"
                        self._durum["decision_log"] = f"CONFIRMED: {durum_str}"
                        self._log(f"SUCCESS: VEHICLE IS NOW {durum_str}")

                if self._durum.get("mode_change_pending"):
                    if self._durum.get("requested_mode") == mod_id:
                        self._durum["mode_change_pending"] = False
                        self._durum["requested_mode"] = -1
                        self._mode_change_requested_at = 0.0
                        self._durum["decision_log"] = f"CONFIRMED: {mod_name} MODE ACTIVE"
                        self._log(f"SUCCESS: MODE CHANGED TO {mod_name}")

            heartbeat_updates = dict(
                armed=armed,
                mod_id=mod_id,
                mod=mod_name,
                system_status=system_status,
                baglanti=True,
                link_ok=True,
                heartbeat_seen=True,
                telemetry_lost=False,
            )
            if not armed:
                # Pixhawk can keep reporting a small/noisy GPS groundspeed while
                # the motors are disarmed. Keep displayed motion at zero until
                # a later heartbeat confirms that the vehicle is armed.
                self._filtered_cog = None
                heartbeat_updates.update(hiz=0.0, cog=0.0)
            self._set(**heartbeat_updates)

        elif msg_type == "VFR_HUD":
            if not self._kilitli_arac_kaynagi_mi(msg):
                return
            if not self._telemetri_saglikli_mi():
                self._set(**self._telemetri_degerlerini_sifirla())
                return
            with self._lock:
                armed = bool(self._durum.get("armed"))
                disarm_requested = bool(self._durum.get("arm_change_pending")) and not bool(
                    self._durum.get("requested_arm_state")
                )
            # SOG is updated only from GPS_RAW_INT.vel below. VFR_HUD may be
            # relayed by other MAVLink participants and must not drive speed.
            updates = {"yaw": msg.heading}
            if not armed or disarm_requested:
                updates["hiz"] = 0.0
                self._filtered_cog = None
                self._filtered_sog = 0.0
                updates["cog"] = 0.0
            self._set(**updates)

        elif msg_type == "ATTITUDE":
            self._set(
                roll=math.degrees(msg.roll),
                pitch=math.degrees(msg.pitch),
                yaw=(math.degrees(msg.yaw) + 360.0) % 360.0,
            )

        elif msg_type in ("AHRS", "AHRS2"):
            updates = {}
            if hasattr(msg, "roll"):
                updates["roll"] = math.degrees(msg.roll)
            if hasattr(msg, "pitch"):
                updates["pitch"] = math.degrees(msg.pitch)
            if hasattr(msg, "yaw"):
                updates["yaw"] = (math.degrees(msg.yaw) + 360.0) % 360.0
            lat_raw = getattr(msg, "lat", None)
            lon_raw = getattr(msg, "lng", getattr(msg, "lon", None))
            if lat_raw is not None and lon_raw is not None:
                lat = float(lat_raw) / 1e7 if abs(float(lat_raw)) > 1000 else float(lat_raw)
                lon = float(lon_raw) / 1e7 if abs(float(lon_raw)) > 1000 else float(lon_raw)
                if -90 <= lat <= 90 and -180 <= lon <= 180:
                    updates["lat"] = lat
                    updates["lon"] = lon
            if updates:
                self._set(**updates)

        elif msg_type == "NAV_CONTROLLER_OUTPUT":
            self._set(mesafe=msg.wp_dist)

        elif msg_type == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            updates = {"lat": lat, "lon": lon}
            hdg = getattr(msg, "hdg", 65535)
            if hdg != 65535:
                updates["yaw"] = float(hdg) / 100.0
            self._set(**updates)

        elif msg_type == "GPS_RAW_INT":
            if not self._kilitli_arac_kaynagi_mi(msg):
                return
            updates = {
                "gps": msg.fix_type,
                "gps_uydu": msg.satellites_visible,
            }
            with self._lock:
                armed = bool(self._durum.get("armed"))
                disarm_requested = bool(self._durum.get("arm_change_pending")) and not bool(
                    self._durum.get("requested_arm_state")
                )
            cog_raw = getattr(msg, "cog", 65535)
            velocity_raw = getattr(msg, "vel", 65535)
            if velocity_raw != 65535:
                speed = self._sog_filtrele(float(velocity_raw) / 100.0)
            else:
                with self._lock:
                    speed = float(self._durum.get("hiz", 0.0) or 0.0)
            if not armed or disarm_requested:
                speed = 0.0
                updates["hiz"] = 0.0
                self._filtered_cog = None
                self._filtered_sog = 0.0
                updates["cog"] = 0.0
            elif int(getattr(msg, "fix_type", 0)) < 3:
                speed = 0.0
                updates["hiz"] = 0.0
                self._filtered_cog = None
                self._filtered_sog = 0.0
                updates["cog"] = 0.0
            else:
                updates["hiz"] = speed
            if (
                    armed
                    and not disarm_requested
                    and cog_raw != 65535
                    and int(getattr(msg, "fix_type", 0)) >= 3
            ):
                updates["cog"] = self._cog_filtrele(float(cog_raw) / 100.0, speed)
            elif speed < 0.30:
                self._filtered_cog = None
                updates["cog"] = 0.0
            self._set(**updates)

        elif msg_type == "SYS_STATUS":
            with self._lock:
                battery = self._battery_state()
                raw_voltage = msg.voltage_battery / 1000.0
                if self._filtered_battery_voltage is None:
                    self._filtered_battery_voltage = raw_voltage
                else:
                    self._filtered_battery_voltage += (
                        raw_voltage - self._filtered_battery_voltage
                    ) * BATTERY_VOLTAGE_FILTER_ALPHA
                voltage = self._filtered_battery_voltage
                battery["total_voltage"] = voltage
                try:
                    reported_percent = int(getattr(msg, "battery_remaining", -1))
                except (TypeError, ValueError):
                    reported_percent = -1
                percent = _pil_yuzdesi_voltajdan(voltage)
                battery["reported_percentage"] = (
                    reported_percent if 0 <= reported_percent <= 100 else None
                )
                battery["percentage_source"] = "6s_lipo_voltage_estimate"
                battery["percentage"] = percent
                battery["remaining_wh"] = BATTERY_CAPACITY_WH * percent / 100.0
            self._emit_durum()

        elif msg_type == "COMMAND_ACK":
            command = getattr(msg, "command", None)
            result = getattr(msg, "result", None)
            command_name = self._mavlink_command_name(command)
            result_name = self._mavlink_result_name(result)
            self._log(f"COMMAND ACK: {command_name} -> {result_name}")
            command_value = int(command if command is not None else -1)
            result_value = int(result if result is not None else -1)
            if command_value == mavutil.mavlink.MAV_CMD_DO_SET_RELAY:
                with self._lock:
                    relay_pending = bool(
                        self._durum.get("emergency_relay_pending")
                    )
                    requested_state = self._emergency_relay_requested_state

                if not relay_pending or requested_state is None:
                    self._log(
                        "INFO: Ignoring DO_SET_RELAY ACK because no relay "
                        "command is pending."
                    )
                elif not self._kilitli_arac_kaynagi_mi(msg):
                    self._log(
                        "WARNING: Ignoring DO_SET_RELAY ACK from a source "
                        "other than the locked Pixhawk."
                    )
                elif result_value == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    self._log(
                        "INFO: Pixhawk reports relay command in progress; "
                        "waiting for final confirmation."
                    )
                elif result_value == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    relay_active = bool(requested_state)
                    expected_aux_high = (
                        relay_active != EMERGENCY_RELAY_INVERTED
                    )
                    self._emergency_relay_requested_at = 0.0
                    self._emergency_relay_requested_state = None
                    self._set(
                        emergency_relay_active=relay_active,
                        emergency_relay_pending=False,
                        decision_log=(
                            "Emergency active: vehicle power cut."
                            if relay_active
                            else "Safe start active: vehicle power enabled."
                        ),
                    )
                    self._log(
                        "Pixhawk accepted AUX OUT 5 relay command; "
                        f"expected physical level is "
                        f"{'HIGH' if expected_aux_high else 'LOW'}."
                    )
                else:
                    self._emergency_relay_requested_at = 0.0
                    self._emergency_relay_requested_state = None
                    self._set(
                        emergency_relay_pending=False,
                        decision_log=(
                            "Emergency relay command rejected by Pixhawk; "
                            "relay state was not changed."
                        ),
                    )

        elif msg_type == "TUNNEL":
            payload_type = int(getattr(msg, "payload_type", -1))
            try:
                payload_length = int(getattr(msg, "payload_length", 0))
                payload = bytes(msg.payload)[:payload_length]
            except (AttributeError, TypeError, ValueError) as exc:
                self._log(f"WARNING: Invalid MAVLink TUNNEL payload: {exc}")
                return

            if payload_type == DETECTION_PAYLOAD_TYPE:
                try:
                    detection = decode_detection_payload(payload)
                except ValueError as exc:
                    self._log(f"WARNING: Invalid detection MAVLink payload: {exc}")
                    return

                detection["received_monotonic"] = time.monotonic()
                with self._lock:
                    detections = list(self._durum.get("detections", []))

                    # TUNNEL sends one message per object. Keep messages belonging
                    # to the same camera frame together, but discard the previous
                    # frame as soon as the first object from a new frame arrives.
                    current_frame_id = detections[-1].get("frame_id") if detections else None
                    if current_frame_id != detection.get("frame_id"):
                        detections = []

                    # Protect the UI against an accidentally repeated MAVLink
                    # packet without hiding separate objects from the same frame.
                    sequence = detection.get("sequence")
                    if any(item.get("sequence") == sequence for item in detections):
                        return
                    detections.append(detection)
                    detections = detections[-DETECTION_FRAME_LIMIT:]
                    self._durum["detections"] = detections
                    self._durum["detection"] = max(
                        detections,
                        key=lambda item: float(item.get("confidence", 0.0)),
                    )
                self._emit_durum()

            elif payload_type == DECISION_PAYLOAD_TYPE:
                try:
                    decision = decode_decision_payload(payload)
                except ValueError as exc:
                    self._log(f"WARNING: Invalid decision MAVLink payload: {exc}")
                    return
                decision["received_monotonic"] = time.monotonic()
                with self._lock:
                    self._durum["mission_decision"] = decision
                    self._durum["active_mission"] = decision.get("active_mission")
                self._emit_durum()

        elif msg_type == "STATUSTEXT":
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode(errors="replace")
            text = str(text).strip("\x00").strip()
            if text:
                if text.upper().startswith("JETSON:"):
                    self._log(text)
                else:
                    self._log(f"PIXHAWK: {text}")
                lower_text = text.lower()
                if "flight mode change failed" in lower_text:
                    self._mode_degisim_red_nedenlerini_logla()
                if "radio failsafe" in lower_text:
                    self._last_radio_failsafe = time.time()
                    self._set(
                        radio_failsafe=True,
                        system_status="FAILSAFE_RADIO",
                        decision_log=(
                            "RADIO FAILSAFE ACTIVE: Mission start is blocked. "
                            "Check RC receiver or Pixhawk failsafe settings in Mission Planner."
                        ),
                    )

        elif msg_type in (
            "MISSION_COUNT",
            "MISSION_ITEM",
            "MISSION_ITEM_INT",
            "MISSION_REQUEST",
            "MISSION_REQUEST_INT",
            "MISSION_ACK",
            "PARAM_VALUE",
        ):
            with self._lock:
                self._mission_messages.append(msg)
                self._mission_messages = self._mission_messages[-30:]

    @staticmethod
    def _arac_heartbeat_mi(msg):
        """Yalnizca gercek otopilot heartbeat'ini arac durumu olarak kabul et."""
        mavlink = mavutil.mavlink
        vehicle_type = getattr(msg, "type", None)
        autopilot = getattr(msg, "autopilot", None)

        ignored_types = {
            getattr(mavlink, "MAV_TYPE_GCS", None),
            getattr(mavlink, "MAV_TYPE_ONBOARD_CONTROLLER", None),
        }
        if vehicle_type in ignored_types:
            return False
        if autopilot in (None, getattr(mavlink, "MAV_AUTOPILOT_INVALID", None)):
            return False
        return True

    def _arac_heartbeat_kaynagini_kabul_et(self, msg):
        source_system = int(getattr(msg, "get_srcSystem", lambda: 0)() or 0)
        source_component = int(getattr(msg, "get_srcComponent", lambda: 0)() or 0)
        source = (source_system, source_component)

        if not self._arac_heartbeat_mi(msg):
            if source not in self._ignored_heartbeat_sources:
                self._ignored_heartbeat_sources.add(source)
                self._log(
                    "Ignoring non-vehicle heartbeat: "
                    f"system={source_system}, component={source_component}"
                )
            return False

        if self._vehicle_system_id is None:
            self._vehicle_system_id = source_system
            self._vehicle_component_id = source_component
            if self.connection is not None:
                self.connection.target_system = source_system
                self.connection.target_component = source_component
            self._log(
                "Pixhawk heartbeat source locked: "
                f"system={source_system}, component={source_component}"
            )
            return True

        return source == (self._vehicle_system_id, self._vehicle_component_id)

    def _kilitli_arac_kaynagi_mi(self, msg):
        """Hız/GPS telemetrisini yalnızca kilitlenmiş Pixhawk kaynağından kabul et."""
        if self._vehicle_system_id is None or self._vehicle_component_id is None:
            return False
        source_system = int(getattr(msg, "get_srcSystem", lambda: 0)() or 0)
        source_component = int(getattr(msg, "get_srcComponent", lambda: 0)() or 0)
        return (source_system, source_component) == (
            self._vehicle_system_id,
            self._vehicle_component_id,
        )

    def _mavlink_streamlerini_iste(self):
        if not self.connection:
            return

        self._streams_requested = True
        try:
            target_system = self.connection.target_system or 1

            for stream_id in (
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            ):
                self.connection.mav.request_data_stream_send(
                    target_system,
                    0,
                    stream_id,
                    2,
                    1,
                )

            self._set(decision_log="MAVLink telemetry streams requested at low rate.")
            self._log("MAVLink telemetry streams requested at low rate.")
        except Exception as exc:
            self._log(f"ERROR: MAVLink stream request failed: {exc}")

    def _cog_filtrele(self, candidate, speed):
        """COG'u dururken sıfırla, hareket halinde dairesel olarak yumuşat."""
        if not math.isfinite(speed) or speed < 0.30:
            self._filtered_cog = None
            return 0.0

        candidate = float(candidate) % 360.0
        if self._filtered_cog is None:
            self._filtered_cog = candidate
            return candidate

        # Normal ortalama 359° ile 1°'yi yanlışlıkla 180° yapar. En kısa açı
        # farkını kullanmak kuzey geçişlerinde de doğru sonuç verir.
        difference = (candidate - self._filtered_cog + 180.0) % 360.0 - 180.0
        self._filtered_cog = (self._filtered_cog + difference * 0.25) % 360.0
        return self._filtered_cog

    def _sog_filtrele(self, candidate):
        """GPS SOG değerini doğrula ve ekrandaki ani sıçramaları yumuşat."""
        try:
            candidate = float(candidate)
        except (TypeError, ValueError):
            candidate = float("nan")

        if (
                not math.isfinite(candidate)
                or candidate < 0.0
                or candidate > MAX_VALID_SOG_M_S
        ):
            now = time.time()
            if now - self._last_sog_reject_log > 2.0:
                self._last_sog_reject_log = now
                self._log(
                    "WARNING: Invalid GPS SOG rejected: "
                    f"{candidate!r} m/s (limit {MAX_VALID_SOG_M_S:.1f} m/s)"
                )
            return float(self._filtered_sog or 0.0)

        if candidate < 0.05:
            candidate = 0.0
        previous = float(self._filtered_sog or 0.0)
        self._filtered_sog = (
            candidate
            if previous <= 0.0
            else previous + (candidate - previous) * SOG_FILTER_ALPHA
        )
        return self._filtered_sog

    def _mesaj_araliklarini_iste(self, target_system, target_component):
        mesajlar = {
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT: 1,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: 5,
            mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: 5,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 10,
        }

        for message_id, hz in mesajlar.items():
            interval_us = int(1_000_000 / hz)
            self.connection.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )

    def _mavlink_command_name(self, command):
        try:
            return mavutil.mavlink.enums["MAV_CMD"][int(command)].name
        except Exception:
            return str(command)

    def _mavlink_result_name(self, result):
        try:
            return mavutil.mavlink.enums["MAV_RESULT"][int(result)].name
        except Exception:
            return str(result)

    def _mavlink_state_name(self, state):
        try:
            return mavutil.mavlink.enums["MAV_STATE"][int(state)].name
        except Exception:
            return str(state)

    def _komut_gonder(
        self,
        komut_id,
        p1=0.0,
        p2=0.0,
        p3=0.0,
        p4=0.0,
        p5=0.0,
        p6=0.0,
        p7=0.0,
        target_component=None,
    ):
        if not self.connection:
            self._log("ERROR: No connection. Command was not sent.")
            return False

        target_system = self.connection.target_system or 1
        target_component = target_component
        if target_component is None:
            target_component = self.connection.target_component or 1

        self.connection.mav.command_long_send(
            target_system,
            target_component,
            komut_id,
            0,
            p1,
            p2,
            p3,
            p4,
            p5,
            p6,
            p7,
        )
        return True

    def _mavlink_hedefleri(self, component_zero=False):
        target_system = self.connection.target_system or 1
        if component_zero:
            target_component = 0
        else:
            target_component = self.connection.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        return target_system, target_component

    def _arm_disarm_gonder(self, armed, force=False, repeat=1):
        if not self.connection:
            self._log("ERROR: No connection. ARM/DISARM command was not sent.")
            return False

        target_system, target_component = self._mavlink_hedefleri(component_zero=False)
        force_param = float(ARDUPILOT_FORCE_ARM_MAGIC) if armed and force else 0.0
        for _ in range(max(1, int(repeat))):
            self.connection.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0 if armed else 0.0,
                force_param,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            time.sleep(0.15)
        return True

    def arm_yap(self):
        self._arkaplan_calistir("MAVLinkArm", self._arm_yap_mavlink)

    def _arm_yap_mavlink(self):
        with self._lock:
            if self._durum["mod"] == "EMERGENCY":
                self._log("ERROR: Reset emergency state before arming.")
                return
            if self._durum["armed"]:
                self._log("INFO: Vehicle is already armed.")
                return

        hold_mode_id = MODE_NAME_TO_ID["HOLD"]
        self._log("SAFETY: ARM requested from GUI. Forcing HOLD before arming.")
        if not self._mod_ayarla_mavlink(hold_mode_id, replace_pending=True):
            self._log("ERROR: ARM aborted because HOLD mode command could not be sent.")
            return
        time.sleep(0.2)

        if self._arm_disarm_gonder(armed=True, force=True, repeat=3):
            self._set(
                requested_arm_state=True,
                arm_change_pending=True,
                requested_mode=hold_mode_id,
                mode_change_pending=True,
                decision_log="GUI ARM requested. HOLD command sent first, then FORCE ARM. Waiting for heartbeat confirmation...",
            )
            self._log("MAVLink FORCE ARM command sent after HOLD -> waiting for heartbeat confirmation")

    def disarm_yap(self):
        self._arkaplan_calistir("MAVLinkDisarm", self._disarm_yap_mavlink)

    def _disarm_yap_mavlink(self):
        if self._arm_disarm_gonder(armed=False, force=False, repeat=2):
            self._set(
                requested_arm_state=False,
                arm_change_pending=True,
                hiz=0.0,
                decision_log="MAVLink DISARM command sent. Waiting for heartbeat confirmation...",
            )
            self._log("MAVLink DISARM command sent -> waiting for heartbeat confirmation")

    def mod_ayarla(self, mod_id):
        self._arkaplan_calistir("MAVLinkMode", self._mod_ayarla_mavlink, mod_id)

    def mod_ayarla_ad(self, mod_name):
        mode_id = MODE_NAME_TO_ID.get(str(mod_name).upper())
        if mode_id is None:
            self._log(f"ERROR: Unknown mode selected: {mod_name}")
            return
        self.mod_ayarla(mode_id)

    def _mode_on_kosullari_uygun_mu(self, mod_id):
        mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")
        if mod_name not in ("AUTO", "GUIDED"):
            return True

        with self._lock:
            armed = bool(self._durum.get("armed"))
            gps_fix = int(self._durum.get("gps", 0) or 0)
            lat = float(self._durum.get("lat", 0.0) or 0.0)
            lon = float(self._durum.get("lon", 0.0) or 0.0)
            mission_ok = bool(self._mission_uploaded_to_pixhawk)

        eksikler = []
        if not self._telemetri_saglikli_mi():
            eksikler.append("telemetry link is not healthy")
        if not armed:
            eksikler.append("vehicle is not armed")
        if gps_fix < 3:
            eksikler.append(f"GPS fix is {gps_fix}; 3D fix is required")
        if abs(lat) < 0.000001 and abs(lon) < 0.000001:
            eksikler.append("vehicle position is not valid")
        if mod_name == "AUTO" and not mission_ok:
            eksikler.append("mission is not verified on Pixhawk")

        if not eksikler:
            return True

        detay = "; ".join(eksikler)
        self._set(
            mode_change_pending=False,
            requested_mode=-1,
            decision_log=f"{mod_name} mode blocked: {detay}.",
        )
        self._log(f"WARNING: {mod_name} MODE command blocked: {detay}.")
        return False

    def _mod_ayarla_mavlink(self, mod_id, replace_pending=False):
        mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")
        if not self.connection:
            self._log("ERROR: No connection for mode change.")
            return False
        with self._lock:
            if self._durum.get("mode_change_pending") and not replace_pending:
                self._log("INFO: A mode change is already waiting for Pixhawk confirmation.")
                return False
        if not self._mode_on_kosullari_uygun_mu(mod_id):
            return False

        try:
            target_system = self.connection.target_system or 1
            self.connection.mav.set_mode_send(
                target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mod_id,
            )
            self._mode_change_requested_at = time.monotonic()
            self._set(
                requested_mode=mod_id,
                mode_change_pending=True,
                decision_log=f"MAVLink MODE command sent: {mod_name}. Waiting for heartbeat confirmation...",
            )
            self._log(f"MAVLink MODE command sent: {mod_name} ({mod_id})")
            return True
        except Exception as exc:
            self._mode_change_requested_at = 0.0
            self._set(
                mode_change_pending=False,
                requested_mode=-1,
                decision_log=f"{mod_name} mode command could not be sent: {exc}",
            )
            self._log(f"ERROR: MAVLink MODE command failed: {exc}")
            return False

    def _mode_degisim_red_nedenlerini_logla(self):
        simdi = time.time()
        if simdi - self._last_mode_failure_diagnostic < 1.0:
            return
        self._last_mode_failure_diagnostic = simdi

        with self._lock:
            requested_mode = self._durum.get("requested_mode", -1)
            requested_mode_name = ARDUROVER_MODS.get(requested_mode, "UNKNOWN")
            armed = bool(self._durum.get("armed"))
            gps_fix = int(self._durum.get("gps", 0) or 0)
            gps_uydu = int(self._durum.get("gps_uydu", 0) or 0)
            lat = float(self._durum.get("lat", 0.0) or 0.0)
            lon = float(self._durum.get("lon", 0.0) or 0.0)
            telemetry_lost = bool(self._durum.get("telemetry_lost"))
            radio_failsafe = bool(self._durum.get("radio_failsafe"))
            mission_ok = bool(self._mission_uploaded_to_pixhawk)

        konum_var = abs(lat) >= 0.000001 or abs(lon) >= 0.000001
        eksikler = []
        if telemetry_lost:
            eksikler.append("telemetry link not healthy")
        if radio_failsafe:
            eksikler.append("radio failsafe active")
        if requested_mode_name in ("AUTO", "GUIDED") and not armed:
            eksikler.append("vehicle is not armed")
        if requested_mode_name == "AUTO" and not mission_ok:
            eksikler.append("mission is not verified on Pixhawk")
        if requested_mode_name in ("AUTO", "GUIDED") and gps_fix < 3:
            eksikler.append(f"GPS fix is {gps_fix}, needs 3D fix")
        if requested_mode_name in ("AUTO", "GUIDED") and not konum_var:
            eksikler.append("vehicle position/home is not valid")

        if eksikler:
            detay = "; ".join(eksikler)
            self._log(f"MODE CHANGE BLOCKED CHECK ({requested_mode_name}): {detay}.")
            decision_log = f"{requested_mode_name} rejected by Pixhawk: {detay}."
        else:
            self._log(
                "MODE CHANGE BLOCKED CHECK "
                f"({requested_mode_name}): GUI prerequisites look OK "
                f"(armed={armed}, gps_fix={gps_fix}, sats={gps_uydu}, "
                f"lat={lat:.7f}, lon={lon:.7f}, mission_verified={mission_ok}). "
                "Check Pixhawk pre-arm/failsafe/EKF messages in Mission Planner."
            )
            decision_log = (
                f"{requested_mode_name} rejected by Pixhawk. GUI prerequisites look OK; "
                "check Pixhawk EKF, failsafe, and mode settings."
            )
        self._mode_change_requested_at = 0.0
        self._set(
            mode_change_pending=False,
            requested_mode=-1,
            decision_log=decision_log,
        )

    def _mission_mesaji_bekle(self, beklenen_tipler, timeout=8.0):
        deadline = time.time() + timeout
        beklenen_tipler = set(beklenen_tipler)
        while time.time() < deadline:
            if not self._mavlink_gorev_baglantisi_hazir_mi():
                return None
            with self._lock:
                for index, msg in enumerate(self._mission_messages):
                    if msg.get_type() in beklenen_tipler:
                        return self._mission_messages.pop(index)
            time.sleep(0.05)
        return None

    def _mission_kuyrugunu_temizle(self):
        with self._lock:
            self._mission_messages.clear()

    def _waypoints_dosyasini_oku(self, waypoints_yolu):
        waypoints_yolu = Path(waypoints_yolu)
        if waypoints_yolu.suffix.lower() != ".waypoints":
            raise ValueError("Mission file must have a .waypoints extension.")
        text = waypoints_yolu.read_text(encoding="utf-8", errors="replace")
        waypoints = []
        qgc_wpl = False
        for line_no, line in enumerate(text.splitlines(), start=1):
            temiz = line.strip()
            if not temiz or temiz.startswith("#"):
                continue

            if temiz.upper().startswith("QGC WPL"):
                qgc_wpl = True
                continue

            if qgc_wpl:
                waypoint = self._qgc_wpl_satiri_oku(temiz, len(waypoints) + 1, line_no)
                if waypoint is not None:
                    waypoints.append(waypoint)
                continue

            sayilar = re.findall(r"[-+]?\d+(?:\.\d+)?", temiz)
            if len(sayilar) < 2:
                continue

            candidates = []
            for index in range(len(sayilar) - 1):
                try:
                    lat = float(sayilar[index])
                    lon = float(sayilar[index + 1])
                except ValueError:
                    continue
                if -90 <= lat <= 90 and -180 <= lon <= 180:
                    candidates.append((index, lat, lon))

            if not candidates:
                continue

            _index, lat, lon = self._en_olasi_lat_lon_cifti(candidates)
            waypoints.append(
                {
                    "name": f"WP_{len(waypoints) + 1:02d}",
                    "lat": lat,
                    "lon": lon,
                    "alt": 100.0,
                    "line": line_no,
                }
            )

        if not waypoints:
            raise ValueError("No valid latitude/longitude waypoint found in .waypoints file.")
        return waypoints

    def _txt_waypointlerini_oku(self, txt_yolu):
        """Geriye dönük uyumluluk için eski okuyucu adı."""
        return self._waypoints_dosyasini_oku(txt_yolu)

    def _qgc_wpl_satiri_oku(self, line, waypoint_index, line_no):
        parts = re.split(r"[\t,; ]+", line.strip())
        if len(parts) < 12:
            return None

        try:
            seq = int(float(parts[0]))
            current = int(float(parts[1]))
            frame = int(float(parts[2]))
            command = int(float(parts[3]))
            param1, param2, param3, param4 = (float(value) for value in parts[4:8])
            lat = float(parts[8])
            lon = float(parts[9])
            alt = float(parts[10])
            autocontinue = int(float(parts[11]))
        except (TypeError, ValueError, IndexError):
            return None

        if command != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            return None
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return None

        is_home = seq == 0 and current == 1
        return {
            "name": "HOME" if is_home else f"WP_{seq:02d}",
            "seq": seq,
            "current": current,
            "frame": frame,
            "command": command,
            "param1": param1,
            "param2": param2,
            "param3": param3,
            "param4": param4,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "autocontinue": autocontinue,
            "is_home": is_home,
            "line": line_no,
        }

    def _en_olasi_lat_lon_cifti(self, candidates):
        def score(item):
            index, lat, lon = item
            value = index * 0.01
            if 35.0 <= abs(lat) <= 72.0:
                value += 4.0
            if -20.0 <= lon <= 45.0:
                value += 4.0
            if abs(lat) >= abs(lon):
                value += 1.0
            return value

        return max(candidates, key=score)

    def _mission_count_gonder(self, target_system, target_component, count):
        try:
            self.connection.mav.mission_count_send(
                target_system,
                target_component,
                count,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_count_send(target_system, target_component, count)

    def _mission_item_gonder(self, target_system, target_component, seq, waypoint, use_int=True):
        lat_int = int(float(waypoint["lat"]) * 1e7)
        lon_int = int(float(waypoint["lon"]) * 1e7)
        command = int(waypoint.get("command", mavutil.mavlink.MAV_CMD_NAV_WAYPOINT))
        current = int(waypoint.get("current", 0))
        autocontinue = int(waypoint.get("autocontinue", 1))
        frame = int(waypoint.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))
        int_frames = {
            mavutil.mavlink.MAV_FRAME_GLOBAL: mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT: mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        }
        int_frame = int_frames.get(frame, frame)
        params = [float(waypoint.get(f"param{index}", 0.0) or 0.0) for index in range(1, 5)]
        altitude = float(waypoint.get("alt", waypoint.get("altitude", 0.0)) or 0.0)

        if not use_int:
            self._mission_item_float_gonder(
                target_system, target_component, seq, waypoint, frame,
                command, current, autocontinue, params, altitude,
            )
            return

        args = (
            target_system, target_component, seq, int_frame, command,
            current, autocontinue, *params, lat_int, lon_int, altitude,
        )
        try:
            self.connection.mav.mission_item_int_send(
                *args, mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_item_int_send(*args)

    def _mission_item_float_gonder(
        self, target_system, target_component, seq, waypoint, frame,
        command, current, autocontinue, params, altitude,
    ):
        args = (
            target_system, target_component, seq, frame, command,
            current, autocontinue, *params,
            float(waypoint["lat"]), float(waypoint["lon"]), altitude,
        )
        try:
            self.connection.mav.mission_item_send(
                *args, mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_item_send(*args)

    def _mavlink_gorev_baglantisi_hazir_mi(self):
        with self._lock:
            baglanti = bool(self._durum.get("baglanti"))
            link_ok = bool(self._durum.get("link_ok"))
            heartbeat_seen = bool(self._durum.get("heartbeat_seen"))
            telemetry_lost = bool(self._durum.get("telemetry_lost"))

        return bool(self.connection and baglanti and link_ok and heartbeat_seen and not telemetry_lost)

    def arac_bagli_mi(self):
        return self._mavlink_gorev_baglantisi_hazir_mi()

    def _mavlink_gorev_yukle(self, waypoints):
        if not self._mavlink_gorev_baglantisi_hazir_mi():
            raise RuntimeError(
                "No healthy Pixhawk MAVLink telemetry connection. Use CONNECT and wait for heartbeat first."
            )

        son_hata = None
        for component_zero in (False, True):
            try:
                self._mavlink_gorev_yukle_hedef(waypoints, component_zero=component_zero)
                self._mission_component_zero = component_zero
                return
            except RuntimeError as exc:
                son_hata = exc
                if not component_zero:
                    self._log(f"WARNING: Mission upload retrying with target component 0 after: {exc}")
                    time.sleep(0.5)
                    continue
                raise
        if son_hata:
            raise son_hata

    def _mission_items_hazirla(self, waypoints):
        items = list(waypoints)
        if items and items[0].get("is_home"):
            return items
        return [self._home_waypoint_yap(items)] + items

    def _mavlink_gorev_yukle_hedef(self, waypoints, component_zero=False):
        target_system, target_component = self._mavlink_hedefleri(component_zero=component_zero)
        mission_items = self._mission_items_hazirla(waypoints)
        self._mission_kuyrugunu_temizle()

        try:
            self.connection.mav.mission_clear_all_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_clear_all_send(target_system, target_component)

        clear_ack = self._mission_mesaji_bekle(("MISSION_ACK",), timeout=2.0)
        if clear_ack is not None:
            result = getattr(clear_ack, "type", None)
            self._log(f"MAVLink mission clear ACK: {self._mavlink_mission_result_name(result)}")

        self._mission_kuyrugunu_temizle()
        self._mission_count_gonder(target_system, target_component, len(mission_items))
        self._log(
            f"MAVLink mission upload started: {len(mission_items)} mission item(s), "
            f"target_component={target_component}"
        )

        gonderilenler = set()
        resend_count = 0
        deadline = time.time() + max(12.0, len(mission_items) * 3.0)
        while time.time() < deadline:
            msg = self._mission_mesaji_bekle(
                ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"),
                timeout=2.0,
            )
            if msg is None:
                if not self._mavlink_gorev_baglantisi_hazir_mi():
                    raise RuntimeError("Telemetry link was lost during mission upload.")
                if not gonderilenler:
                    resend_count += 1
                    self._mission_count_gonder(target_system, target_component, len(mission_items))
                    self._log(f"MAVLink mission count resent ({resend_count})")
                continue

            msg_type = msg.get_type()
            if msg_type == "MISSION_ACK":
                result = getattr(msg, "type", None)
                result_name = self._mavlink_mission_result_name(result)
                if len(gonderilenler) < len(mission_items):
                    self._log(f"INFO: Ignoring early mission ACK before all waypoints were sent: {result_name}")
                    if not gonderilenler:
                        self._mission_count_gonder(target_system, target_component, len(mission_items))
                    continue
                if int(result or 0) == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    self._log("MAVLink mission upload accepted by Pixhawk.")
                    return
                raise RuntimeError(f"Mission upload rejected: {result_name}")

            seq = int(getattr(msg, "seq", -1))
            if seq < 0 or seq >= len(mission_items):
                raise RuntimeError(f"Pixhawk requested invalid mission item: {seq}")

            self._mission_item_gonder(
                target_system,
                target_component,
                seq,
                mission_items[seq],
                use_int=True,
            )
            gonderilenler.add(seq)
            if seq == 0:
                self._log(f"MAVLink mission item sent: HOME/{len(mission_items)}")
            else:
                self._log(f"MAVLink mission item sent: WP {seq}/{len(waypoints)}")

        raise RuntimeError("Mission upload timed out before Pixhawk ACK.")

    def _home_waypoint_yap(self, waypoints):
        first = waypoints[0] if waypoints else {"lat": 0.0, "lon": 0.0, "alt": 0.0}
        with self._lock:
            lat = float(self._durum.get("lat", 0.0) or 0.0)
            lon = float(self._durum.get("lon", 0.0) or 0.0)

        if abs(lat) < 0.000001 and abs(lon) < 0.000001:
            lat = float(first.get("lat", 0.0) or 0.0)
            lon = float(first.get("lon", 0.0) or 0.0)

        return {
            "name": "HOME",
            "seq": 0,
            "current": 1,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "param1": 0.0,
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "lat": lat,
            "lon": lon,
            "alt": float(first.get("alt", first.get("altitude", 0.0)) or 0.0),
            "autocontinue": 1,
            "is_home": True,
        }

    def _mission_request_list_gonder(self, target_system, target_component):
        try:
            self.connection.mav.mission_request_list_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_request_list_send(target_system, target_component)

    def _mission_request_item_gonder(self, target_system, target_component, seq):
        try:
            self.connection.mav.mission_request_int_send(
                target_system,
                target_component,
                seq,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except AttributeError:
            self.connection.mav.mission_request_send(target_system, target_component, seq)
        except TypeError:
            self.connection.mav.mission_request_int_send(target_system, target_component, seq)

    def _mission_item_waypoint_yap(self, msg, seq):
        msg_type = msg.get_type()
        if msg_type == "MISSION_ITEM_INT":
            lat = getattr(msg, "x", 0) / 1e7
            lon = getattr(msg, "y", 0) / 1e7
        else:
            lat = getattr(msg, "x", getattr(msg, "lat", 0.0))
            lon = getattr(msg, "y", getattr(msg, "lon", 0.0))

        command = int(getattr(msg, "command", mavutil.mavlink.MAV_CMD_NAV_WAYPOINT))
        if command != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            return None
        if not (-90 <= float(lat) <= 90 and -180 <= float(lon) <= 180):
            return None

        current = int(getattr(msg, "current", 0) or 0)
        is_home = seq == 0
        return {
            "name": "HOME" if is_home else f"WP_{seq:02d}",
            "seq": seq,
            "current": current,
            "frame": int(getattr(msg, "frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)),
            "command": command,
            "param1": float(getattr(msg, "param1", 0.0) or 0.0),
            "param2": float(getattr(msg, "param2", 0.0) or 0.0),
            "param3": float(getattr(msg, "param3", 0.0) or 0.0),
            "param4": float(getattr(msg, "param4", 0.0) or 0.0),
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(getattr(msg, "z", 0.0) or 0.0),
            "autocontinue": int(getattr(msg, "autocontinue", 1) or 0),
            "is_home": is_home,
        }

    def _mission_item_gecerli_mi(self, msg, beklenen_seq):
        msg_type = msg.get_type()
        if msg_type not in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            return False
        if int(getattr(msg, "seq", -1)) != int(beklenen_seq):
            return False
        return True

    def _waypointler_eslesiyor(self, beklenen, okunan, tolerans=0.00001):
        beklenen = [item for item in beklenen if not item.get("is_home")]
        okunan = [item for item in okunan if not item.get("is_home")]
        if len(beklenen) != len(okunan):
            self._log(
                f"ERROR: Mission verify failed. Uploaded {len(beklenen)} waypoint(s), "
                f"Pixhawk returned {len(okunan)}."
            )
            return False

        for index, (exp, got) in enumerate(zip(beklenen, okunan), start=1):
            lat_fark = abs(float(exp["lat"]) - float(got["lat"]))
            lon_fark = abs(float(exp["lon"]) - float(got["lon"]))
            if lat_fark > tolerans or lon_fark > tolerans:
                self._log(
                    f"ERROR: Mission verify mismatch at WP {index}: "
                    f"expected {float(exp['lat']):.7f}, {float(exp['lon']):.7f}; "
                    f"got {float(got['lat']):.7f}, {float(got['lon']):.7f}"
                )
                return False

        return True

    def _mavlink_gorev_oku(self):
        if not self._mavlink_gorev_baglantisi_hazir_mi():
            raise RuntimeError(
                "No healthy Pixhawk MAVLink telemetry connection. Use CONNECT and wait for heartbeat first."
            )

        target_system, target_component = self._mavlink_hedefleri(component_zero=self._mission_component_zero)
        self._mission_kuyrugunu_temizle()
        self._mission_request_list_gonder(target_system, target_component)
        self._log("Reading mission list from Pixhawk...")

        count_msg = self._mission_mesaji_bekle(("MISSION_COUNT",), timeout=6.0)
        if count_msg is None:
            raise RuntimeError("Pixhawk did not return MISSION_COUNT.")

        count = int(getattr(count_msg, "count", 0) or 0)
        waypoints = []
        for seq in range(count):
            self._mission_request_item_gonder(target_system, target_component, seq)
            item_msg = None
            read_deadline = time.time() + 5.0
            while time.time() < read_deadline:
                candidate = self._mission_mesaji_bekle(("MISSION_ITEM_INT", "MISSION_ITEM"), timeout=1.0)
                if candidate is None:
                    if not self._mavlink_gorev_baglantisi_hazir_mi():
                        raise RuntimeError("Telemetry link was lost during mission read-back.")
                    continue
                if self._mission_item_gecerli_mi(candidate, seq):
                    item_msg = candidate
                    break
                self._log(
                    f"INFO: Ignoring stale mission item seq "
                    f"{getattr(candidate, 'seq', None)} while waiting for {seq}."
                )
            if item_msg is None:
                raise RuntimeError(f"Pixhawk did not return mission item {seq}.")
            waypoint = self._mission_item_waypoint_yap(item_msg, seq)
            if waypoint is not None:
                if seq == 0:
                    self._log(
                        f"Mission item read: HOME/{count} -> "
                        f"{waypoint['lat']:.7f}, {waypoint['lon']:.7f}"
                    )
                else:
                    self._log(
                        f"Mission item read: WP {seq}/{count - 1} -> "
                        f"{waypoint['lat']:.7f}, {waypoint['lon']:.7f}"
                    )
                waypoints.append(waypoint)

        self._log(f"Mission read from Pixhawk: {len(waypoints)} waypoint(s)")
        return waypoints

    def _mavlink_mission_result_name(self, result):
        try:
            return mavutil.mavlink.enums["MAV_MISSION_RESULT"][int(result)].name
        except Exception:
            return str(result)

    def gorev_baslat(self, gorev_adi):
        self._arkaplan_calistir("MAVLinkMissionStart", self._gorev_baslat_mavlink, gorev_adi)

    def _gorev_baslat_mavlink(self, gorev_adi):
        if not self._mavlink_gorev_baglantisi_hazir_mi():
            self._log("ERROR: No healthy Pixhawk MAVLink telemetry connection for mission start. Use CONNECT and wait for heartbeat first.")
            return
        if not self._mission_uploaded_to_pixhawk:
            self._log("ERROR: Mission is not confirmed on Pixhawk. Load a .waypoints mission over telemetry before EXECUTE.")
            self._set(
                active_mission=None,
                decision_log="Mission start blocked. Upload a .waypoints file to Pixhawk over telemetry first.",
            )
            return

        mission_number = self._gorev_numarasi(gorev_adi)
        if mission_number is None:
            self._log(f"ERROR: Unknown mission selection: {gorev_adi}")
            return

        if not self._scr_user1_ayarla(mission_number):
            self._log(f"ERROR: SCR_USER1 could not be confirmed for mission: {gorev_adi}")
            return

        self._set(
            active_mission=gorev_adi,
            decision_log=(
                f"Mission selected: {gorev_adi}. Pixhawk mission is loaded. "
                f"SCR_USER1 was confirmed as {mission_number}."
            ),
        )
        self._log(f"MISSION SELECTED VIA SCR_USER1: {gorev_adi}")

    def _gorev_numarasi(self, gorev_adi):
        text = str(gorev_adi or "").strip().upper()
        if text.startswith("M"):
            text = text[1:]
        try:
            number = int(text)
        except ValueError:
            return None
        if 1 <= number <= 4:
            return number
        return None

    def _scr_user1_ayarla(self, mission_number, timeout=4.0):
        if not self.connection:
            return False

        target_system, target_component = self._mavlink_hedefleri(component_zero=False)
        param_id = b"SCR_USER1"
        expected = float(mission_number)
        self._mission_kuyrugunu_temizle()

        try:
            self.connection.mav.param_set_send(
                target_system,
                target_component,
                param_id,
                expected,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            self._log(f"MAVLink PARAM_SET sent: SCR_USER1={expected:.0f}")
        except Exception as exc:
            self._log(f"ERROR: SCR_USER1 PARAM_SET failed: {exc}")
            return False

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._mission_mesaji_bekle(("PARAM_VALUE",), timeout=0.5)
            if msg is None:
                if not self._mavlink_gorev_baglantisi_hazir_mi():
                    self._log("ERROR: SCR_USER1 confirmation stopped because telemetry link was lost.")
                    return False
                continue
            received_id = getattr(msg, "param_id", "")
            if isinstance(received_id, bytes):
                received_id = received_id.decode(errors="replace")
            received_id = str(received_id).strip("\x00").strip()
            if received_id != "SCR_USER1":
                continue
            received_value = float(getattr(msg, "param_value", float("nan")))
            if abs(received_value - expected) <= 0.001:
                self._log(f"SUCCESS: SCR_USER1 confirmed as {received_value:.0f}")
                return True
            self._log(
                f"ERROR: SCR_USER1 verification mismatch: "
                f"expected {expected:.0f}, got {received_value}"
            )
            return False

        self._log("ERROR: SCR_USER1 PARAM_VALUE confirmation timed out.")
        return False

    def _mission_start_gonder(self):
        if not self.connection:
            self._log("ERROR: No connection for mission start.")
            return False

        ok = self._komut_gonder(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            target_component=0,
        )
        if ok:
            self._log("MAVLink MISSION_START command sent.")
        return ok

    def gorev_waypoints_yukle(self, waypoints_yolu, mission_name=None):
        waypoints_yolu = str(waypoints_yolu)
        mission_name = mission_name or Path(waypoints_yolu).stem
        local_waypoints = self._waypoints_dosyasini_oku(waypoints_yolu)
        mission_items = self._mission_items_hazirla(local_waypoints)
        display_waypoints = [item for item in local_waypoints if not item.get("is_home")]
        pixhawk_waypoints = None

        response = {
            "ok": True,
            "success": True,
            "mission_id": mission_name,
            "mission_name": mission_name,
            "message": "Mission waypoints parsed locally.",
            "waypoints": display_waypoints,
            "backend_used": False,
            "pixhawk_uploaded": False,
            "pixhawk_confirmed": False,
        }
        self._mission_id = mission_name

        with self._lock:
            jetson_ip = self._durum.get("jetson_ip")
        if not self._valid_ip(jetson_ip):
            jetson_ip = None

        try:
            self._log(
                "Uploading mission waypoint file to Jetson backend: "
                f"mission={mission_name}, host={jetson_ip or 'configured backend host'}"
            )
            backend_response = self.backend_client.upload_mission_waypoints(
                waypoints_yolu,
                jetson_ip=jetson_ip,
                mission_name=mission_name,
            )
            response["backend_used"] = True
            response["backend_response"] = backend_response
            self._log(
                "SUCCESS: Mission waypoint file saved by Jetson backend: "
                f"{backend_response.get('filename', mission_name)}"
            )
        except Exception as exc:
            response["backend_error"] = str(exc)
            self._log(f"WARNING: Jetson waypoint file upload failed: {exc}")

        if self._mavlink_gorev_baglantisi_hazir_mi():
            try:
                self._log("Uploading mission waypoints directly to Pixhawk over MAVLink telemetry.")
                self._mavlink_gorev_yukle(local_waypoints)
                self._mission_uploaded_to_pixhawk = True
                response["pixhawk_uploaded"] = True
                self._log("SUCCESS: Mission uploaded directly to Pixhawk by GUI.")
                try:
                    pixhawk_waypoints = self._mavlink_gorev_oku()
                    if self._waypointler_eslesiyor(mission_items, pixhawk_waypoints):
                        response["waypoints"] = display_waypoints
                        response["pixhawk_confirmed"] = True
                        self._log("SUCCESS: Mission list verified from Pixhawk by GUI.")
                    else:
                        response["pixhawk_confirmed"] = False
                        response["pixhawk_error"] = "Pixhawk mission verification mismatch."
                        self._log("WARNING: Mission upload ACK was accepted, but read-back verification did not match exactly. Keeping uploaded mission active.")
                except Exception as exc:
                    self._log(f"WARNING: Direct Pixhawk mission read failed: {exc}")
            except Exception as exc:
                self._mission_uploaded_to_pixhawk = False
                response["pixhawk_uploaded"] = False
                response["pixhawk_error"] = str(exc)
                self._log(f"ERROR: Direct Pixhawk mission upload failed: {exc}")
        else:
            self._mission_uploaded_to_pixhawk = False
            response["pixhawk_error"] = "Pixhawk MAVLink telemetry is not connected or heartbeat is not healthy."
            self._log("WARNING: Mission parsed locally, but Pixhawk telemetry heartbeat is not ready. Use CONNECT before uploading to vehicle.")

        if response.get("pixhawk_confirmed") or response.get("pixhawk_uploaded"):
            self.gorev_noktalarini_guncelle(response.get("waypoints") or display_waypoints)
            self._set(
                active_mission=self._mission_id,
                decision_log="Mission uploaded to vehicle and displayed on map.",
            )
        else:
            self.gorev_noktalarini_guncelle([])
            self._set(
                active_mission=None,
                decision_log="Mission waypoints parsed, but vehicle connection is not ready. Route is not displayed.",
            )
            self._log("INFO: Mission waypoints were parsed only. Vehicle/Pixhawk upload was not confirmed, so map route was not displayed.")
        return response

    def gorev_waypoints_onizle(self, waypoints_yolu):
        """Bir mission dosyasını araca göndermeden yerelde ayrıştırır."""
        items = self._waypoints_dosyasini_oku(waypoints_yolu)
        return [item for item in items if not item.get("is_home")]

    def gorev_txt_yukle(self, txt_yolu, mission_name=None):
        """Geriye dönük uyumluluk için eski yükleyici adı."""
        return self.gorev_waypoints_yukle(txt_yolu, mission_name=mission_name)

    def acil_durum(self):
        with self._lock:
            if self._durum.get("emergency_relay_pending"):
                self._log("INFO: Emergency relay command is already pending.")
                return
            target_emergency_active = not bool(
                self._durum.get("emergency_relay_active")
            )
        self._arkaplan_calistir(
            "MAVLinkEmergencyRelay",
            self._acil_durum_mavlink,
            target_emergency_active,
        )

    def _acil_durum_mavlink(self, target_emergency_active):
        if not self.connection or not self._telemetri_saglikli_mi():
            self._log(
                "ERROR: No healthy telemetry connection. "
                "Emergency relay command was not sent."
            )
            return

        target_system, target_component = self._mavlink_hedefleri(
            component_zero=False
        )
        requested_state = bool(target_emergency_active)
        # The connected contactor was verified on hardware: logical relay ON
        # cuts power, while logical relay OFF enables power. RELAY5_INVERTED
        # still determines the expected physical AUX OUT 5 voltage.
        relay_command_on = requested_state
        expected_aux_high = relay_command_on != EMERGENCY_RELAY_INVERTED
        self._emergency_relay_requested_state = requested_state
        self._emergency_relay_requested_at = time.monotonic()
        self._set(
            emergency_relay_pending=True,
            decision_log=(
                "Emergency stop command sent; waiting for Pixhawk confirmation..."
                if requested_state
                else "Safe start command sent; waiting for Pixhawk confirmation..."
            ),
        )
        try:
            self.connection.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                0,
                float(EMERGENCY_RELAY_NUMBER),
                1.0 if relay_command_on else 0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self._log(
                "MAVLink DO_SET_RELAY sent: "
                f"relay={EMERGENCY_RELAY_NUMBER}, "
                f"logical_state={'ON' if relay_command_on else 'OFF'}, "
                f"expected_aux5={'HIGH' if expected_aux_high else 'LOW'}, "
                f"inverted={int(EMERGENCY_RELAY_INVERTED)}."
            )
        except Exception as exc:
            self._emergency_relay_requested_at = 0.0
            self._emergency_relay_requested_state = None
            self._set(
                emergency_relay_pending=False,
                decision_log=f"Emergency relay command failed: {exc}",
            )
            self._log(f"ERROR: Emergency relay command failed: {exc}")

    def durum_al(self):
        return self._snapshot()

    def update_battery(self, battery_data):
        with self._lock:
            self._battery_state().update(battery_data or {})
        self._emit_durum()
