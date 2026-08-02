import re
import socket
import subprocess
import time

import cv2
from PyQt5.QtGui import QImage

JETSON_MAC = "8c:b8:7e:04:20:a9".lower()
PORT = 5000
INITIAL_RECONNECT_DELAY = 1.0
MAX_RECONNECT_DELAY = 10.0
STOP_CHECK_INTERVAL = 0.1


def get_ip_by_mac(target_mac: str) -> str | None:
    normalized_mac = target_mac.lower().replace("-", ":")
    commands = (
        ["arp", "-a"],
        ["ip", "neigh"],
        ["ip", "neighbor"],
        ["netsh", "interface", "ip", "show", "neighbors"],
    )
    for command in commands:
        try:
            result = subprocess.run(command, capture_output=True, text=True, check=False)
        except FileNotFoundError:
            continue
        for line in result.stdout.splitlines():
            normalized_line = line.lower().replace("-", ":")
            if normalized_mac in normalized_line:
                ip_match = re.search(r"\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b", line)
                if ip_match:
                    return ip_match.group()

    return None


def _should_continue(stop_callback) -> bool:
    return stop_callback is None or stop_callback()


def _wait_before_retry(delay: float, stop_callback) -> bool:
    """Wait without making camera shutdown block for the whole retry delay."""
    deadline = time.monotonic() + delay
    while _should_continue(stop_callback):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return True
        time.sleep(min(STOP_CHECK_INTERVAL, remaining))
    return False


def _video_service_is_open(jetson_ip: str) -> bool:
    try:
        with socket.create_connection((jetson_ip, PORT), timeout=0.5):
            return True
    except OSError:
        return False


def _open_capture(url: str):
    cap = cv2.VideoCapture(url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
    if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
    if cap.isOpened():
        return cap
    cap.release()
    return None


def _log(message: str, log_callback) -> None:
    if log_callback:
        log_callback(message)
    else:
        print(message)


# noinspection D
def main(jetson_ip: str, frame_callback=None, log_callback=None, stop_callback=None):
    url = f"http://{jetson_ip}:{PORT}/video_feed"
    cap = None
    connected_once = False
    connection_error_reported = False
    reconnect_delay = INITIAL_RECONNECT_DELAY

    try:
        while _should_continue(stop_callback):
            if cap is None:
                # Checking the TCP port first prevents FFmpeg from printing a
                # "Connection refused" line for every reconnect attempt.
                if _video_service_is_open(jetson_ip):
                    cap = _open_capture(url)

                if cap is None:
                    if not connection_error_reported:
                        if frame_callback is not None:
                            frame_callback(None)
                        message = (
                            "ZED stream connection lost; reconnecting in background..."
                            if connected_once
                            else "ZED stream unavailable; reconnecting in background..."
                        )
                        _log(message, log_callback)
                        connection_error_reported = True

                    if not _wait_before_retry(reconnect_delay, stop_callback):
                        break
                    reconnect_delay = min(reconnect_delay * 2, MAX_RECONNECT_DELAY)
                    continue

            ret, frame = cap.read()
            if not ret:
                cap.release()
                cap = None
                if not connection_error_reported:
                    if frame_callback is not None:
                        frame_callback(None)
                    _log(
                        "Frame was not received; reconnecting in background...",
                        log_callback,
                    )
                    connection_error_reported = True

                if not _wait_before_retry(reconnect_delay, stop_callback):
                    break
                reconnect_delay = min(reconnect_delay * 2, MAX_RECONNECT_DELAY)
                continue

            if not connected_once:
                _log(f"ZED stream connected: {url}", log_callback)
            elif connection_error_reported:
                _log(f"ZED stream reconnected: {url}", log_callback)
            connected_once = True
            connection_error_reported = False
            reconnect_delay = INITIAL_RECONNECT_DELAY

            if frame_callback is not None:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                height, width, channels = rgb.shape
                qimg = QImage(
                    rgb.tobytes(),
                    width,
                    height,
                    channels * width,
                    QImage.Format_RGB888,
                ).copy()
                frame_callback(qimg)
            else:
                cv2.imshow("ZED Stream", frame)
                if cv2.waitKey(1) == 27:
                    break
    finally:
        if cap is not None:
            cap.release()
        if frame_callback is not None:
            frame_callback(None)
        if frame_callback is None:
            cv2.destroyAllWindows()


def start(jetson_ip: str | None = None, frame_callback=None, log_callback=None, stop_callback=None):
    ip = jetson_ip or get_ip_by_mac(JETSON_MAC)
    while ip is None:
        if stop_callback is not None and not stop_callback():
            return
        message = "Jetson IP was not found, retrying..."
        if log_callback:
            log_callback(message)
        else:
            print(message)
        time.sleep(1.0)
        ip = jetson_ip or get_ip_by_mac(JETSON_MAC)

    if ip is None:
        message = "Jetson was not found. Check that it is on the same network."
        if log_callback:
            log_callback(message)
        else:
            print(message)
        return

    if log_callback:
        log_callback(f"Jetson IP: {ip}")
    else:
        print(f"Jetson IP: {ip}")

    main(
        ip,
        frame_callback=frame_callback,
        log_callback=log_callback,
        stop_callback=stop_callback,
    )


if __name__ == "__main__":
    start()
