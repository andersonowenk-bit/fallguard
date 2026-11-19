#!/usr/bin/env python3
"""
pose_bridge.py  --  MacBook camera -> MediaPipe Pose -> Processing (UDP)

FallGuard.pde normally starts this script itself (with --managed) when you
press Run. You can also run it by hand from Terminal.

Sends one line per camera frame to FallGuard.pde on localhost:12000:
    P;x,y,v;x,y,v;...   33 landmarks, x/y normalized 0..1, v = visibility
    N                   camera is running but nobody is in frame

Also sends a small JPEG of each frame to localhost:12002 so the sketch can
show the camera picture, and listens on localhost:12001 for commands coming back from the sketch
(e.g. "RESCUE"). Step 2 will forward those to the Keyestudio tank over BLE.

Usage:
    python3 pose_bridge.py              # default camera, full model
    python3 pose_bridge.py --preview    # also show the camera with dots
    python3 pose_bridge.py --camera 1   # pick a different camera
"""

import argparse
import json
import subprocess
import os
import socket
import struct
import sys
import threading
import time
import urllib.request
from pathlib import Path

try:
    import cv2
    import numpy as np
    import mediapipe as mp
    from mediapipe.tasks import python as mp_tasks
    from mediapipe.tasks.python import vision
except ImportError as e:
    print(f"Missing package ({e.name}). In Terminal run: pip install -r requirements.txt")
    sys.exit(1)

import importlib.util
if importlib.util.find_spec("bleak") is None:   # needed by robot_link.py
    print("Missing package (bleak). In Terminal run: pip install -r requirements.txt")
    sys.exit(1)

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
    "pose_landmarker_{v}/float16/latest/pose_landmarker_{v}.task"
)


def ensure_model(variant: str) -> Path:
    """Download the pose model next to this script the first time."""
    path = Path(__file__).with_name(f"pose_landmarker_{variant}.task")
    if not path.exists():
        print(f"Downloading pose model ({variant}), one time only...")
        tmp = path.with_suffix(".part")
        try:
            urllib.request.urlretrieve(MODEL_URL.format(v=variant), tmp)
        except Exception as e:
            tmp.unlink(missing_ok=True)
            print(f"Model download failed ({e.__class__.__name__}). Check your internet connection, "
                  "or download it in a browser and put it next to pose_bridge.py:")
            print(MODEL_URL.format(v=variant))
            sys.exit(1)
        tmp.rename(path)
    return path


CHUNK = 8000  # macOS refuses local UDP messages over ~9 KB, so pictures go out in pieces


def send_jpeg(sock, data: bytes, frame_id: int, addr) -> None:
    """Send one JPEG as numbered pieces: [frame id: 2 bytes][piece #: 1][piece count: 1][data]."""
    count = (len(data) + CHUNK - 1) // CHUNK
    for i in range(count):
        piece = struct.pack(">HBB", frame_id, i, count) + data[i * CHUNK:(i + 1) * CHUNK]
        try:
            sock.sendto(piece, addr)
        except OSError:
            return  # a dropped picture is fine; never crash over it


MARKER_FILE = Path(__file__).with_name("markers.json")


class MarkerTracker:
    """Finds the two colored paper markers on the rescue tank (F = front, B = back).

    You teach it the colors by clicking each marker in the sketch (key M).
    The colors are saved in markers.json so you only do it once.
    """

    def __init__(self):
        self.colors = {"F": None, "B": None}
        self.last = {"F": None, "B": None}
        self.last_seen = 0.0
        self.kernel = np.ones((3, 3), np.uint8)
        try:
            self.colors.update(json.loads(MARKER_FILE.read_text()))
            if self.colors["F"] and self.colors["B"]:
                print("Tank marker colors loaded")
        except Exception:
            pass

    def clear(self, which: str) -> None:
        self.colors[which] = None
        self.last[which] = None
        MARKER_FILE.write_text(json.dumps(self.colors))
        print("Using a single marker (direction is learned by watching the tank move)")

    def pick(self, which: str, nx: float, ny: float, frame) -> None:
        h, w = frame.shape[:2]
        x, y = int(nx * w), int(ny * h)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        patch = hsv[max(0, y - 4):y + 5, max(0, x - 4):x + 5].reshape(-1, 3)
        hue, sat, val = (int(v) for v in np.median(patch, axis=0))
        self.colors[which] = [hue, sat, val]
        self.last[which] = (nx, ny)          # start tracking exactly where you clicked
        self.last_seen = time.monotonic()
        MARKER_FILE.write_text(json.dumps(self.colors))
        name = "front" if which == "F" else "back"
        print(f"Tank {name} marker color saved")
        if sat < 90:
            print("That spot isn't very colorful; brighter paper will track better")

    def _mask(self, hsv, which, tight=False):
        hue, sat, val = self.colors[which]
        if tight:   # stricter match, used when the marker merges with a similar-colored wall or door
            s_lo, v_lo = max(80, sat - 30), max(50, val - 50)
            lo, hi = hue - 4, hue + 4
        else:
            s_lo, v_lo = max(80, sat - 60), max(50, val - 90)
            lo, hi = hue - 8, hue + 8
        if lo < 0:      # red wraps around the hue circle
            return (cv2.inRange(hsv, (0, s_lo, v_lo), (hi, 255, 255)) |
                    cv2.inRange(hsv, (180 + lo, s_lo, v_lo), (179, 255, 255)))
        if hi > 179:
            return (cv2.inRange(hsv, (lo, s_lo, v_lo), (179, 255, 255)) |
                    cv2.inRange(hsv, (0, s_lo, v_lo), (hi - 180, 255, 255)))
        return cv2.inRange(hsv, (lo, s_lo, v_lo), (hi, 255, 255))

    def _blobs(self, hsv, which, w, h, tight=False):
        """Up to 6 biggest blobs of this marker's color: (x, y, area), x/y normalized."""
        if not self.colors.get(which):
            return []
        mask = cv2.morphologyEx(self._mask(hsv, which, tight), cv2.MORPH_OPEN, self.kernel)
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask)
        max_area = 0.03 * w * h              # a marker on a small tank is never this big (a door, a wall...)
        blobs = [(cents[i][0] / w, cents[i][1] / h, int(stats[i, cv2.CC_STAT_AREA]))
                 for i in range(1, n) if 25 <= stats[i, cv2.CC_STAT_AREA] <= max_area]
        blobs = sorted(blobs, key=lambda b: -b[2])[:6]
        last = self.last.get(which)
        if not tight and last is not None and not any(np.hypot(b[0] - last[0], b[1] - last[1]) < 0.15 for b in blobs):
            return self._blobs(hsv, which, w, h, tight=True)   # nothing near the marker: try the strict match
        return blobs

    def find(self, frame):
        """Return 'T;fx,fy,bx,by;rfx,rfy,rbx,rby'.

        The first group is the tank: a front + back marker pair that sit close
        together (they are on the same small robot). The second group is just the
        biggest blob of each color, drawn as small dots so you can see what the
        camera is latching onto. -1 = not found.
        """
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        fb, bb = self._blobs(hsv, "F", w, h), self._blobs(hsv, "B", w, h)

        if self.colors.get("F") and not self.colors.get("B"):
            # single-marker mode: biggest blob, preferring one near the last position. B = -2 flags this mode.
            best, best_score = None, 0.0
            recent = self.last["F"] is not None and time.monotonic() - self.last_seen < 3.0
            for f in fb:
                score = float(f[2])
                if recent:
                    d = np.hypot(f[0] - self.last["F"][0], f[1] - self.last["F"][1])
                    if d > 0.15:      # a tank can't jump across the room: ignore other red things
                        continue
                    score /= 1.0 + 8.0 * d
                if score > best_score:
                    best, best_score = f, score
            if best:
                self.last["F"], self.last_seen = best[:2], time.monotonic()
            # keep the last position for 3 s so a briefly hidden cup is re-found in the same place
            fx, fy = best[:2] if best else (-1.0, -1.0)
            return f"T;{fx:.4f},{fy:.4f},-2,-2;{fx:.4f},{fy:.4f},-1,-1"

        best, best_score = None, 0.0
        for f in fb:
            for b in bb:
                d = np.hypot(f[0] - b[0], (f[1] - b[1]) * h / w)   # distance in screen widths
                if d > 0.20 or d < 0.005:
                    continue
                score = min(f[2], b[2]) / (1.0 + 10.0 * d)        # big, similar, close together
                if self.last["F"] is not None:                    # prefer near the last position
                    cx, cy = (f[0] + b[0]) / 2, (f[1] + b[1]) / 2
                    lx, ly = (self.last["F"][0] + self.last["B"][0]) / 2, (self.last["F"][1] + self.last["B"][1]) / 2
                    score /= 1.0 + 8.0 * np.hypot(cx - lx, cy - ly)
                if score > best_score:
                    best, best_score = (f, b), score
        if best:
            self.last["F"], self.last["B"] = best[0][:2], best[1][:2]
            (fx, fy), (bx, by) = best[0][:2], best[1][:2]
        else:
            self.last["F"] = self.last["B"] = None
            fx = fy = bx = by = -1.0
        rf = fb[0][:2] if fb else (-1.0, -1.0)
        rb = bb[0][:2] if bb else (-1.0, -1.0)
        return (f"T;{fx:.4f},{fy:.4f},{bx:.4f},{by:.4f};"
                f"{rf[0]:.4f},{rf[1]:.4f},{rb[0]:.4f},{rb[1]:.4f}")


def stop_old_bridges() -> None:
    """Close any leftover copy of this script that is still holding the camera."""
    try:
        out = subprocess.run(["pgrep", "-f", "pose_bridge.py|robot_link.py --managed"], capture_output=True, text=True).stdout
    except Exception:
        return
    for pid in out.split():
        if pid.isdigit() and int(pid) not in (os.getpid(), os.getppid()):
            try:
                os.kill(int(pid), 15)
                print(f"Closed an old bridge that was still running (pid {pid})")
            except OSError:
                pass
    time.sleep(0.5)  # give macOS a moment to release the camera


def pick_camera() -> int:
    """Pick the Mac's own camera, skipping iPhone Continuity Camera and Desk View.

    Uses the camera list from macOS. OpenCV numbers cameras in the same order
    on most Macs; if it picks wrong, press K in the sketch to try the next one.
    """
    if sys.platform != "darwin":
        return 0
    try:
        out = subprocess.run(["system_profiler", "SPCameraDataType", "-json"],
                             capture_output=True, text=True, timeout=10).stdout
        names = [c.get("_name", "?") for c in json.loads(out).get("SPCameraDataType", [])]
    except Exception:
        return 0
    print("Cameras found: " + ", ".join(f"{i}={n}" for i, n in enumerate(names)))
    for i, n in enumerate(names):
        if any(word in n.lower() for word in ("facetime", "built-in", "macbook")):
            return i
    skip = ("iphone", "ipad", "desk view", "continuity")
    for i, n in enumerate(names):
        if not any(word in n.lower() for word in skip):
            return i
    return 0


ROBOT_LINK = ("127.0.0.1", 12003)
_link_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def handle_robot_command(cmd: str) -> None:
    """Pass a command from the sketch to robot_link.py, which owns the Bluetooth link."""
    if cmd != "STOP" and not cmd.startswith("DRIVE"):
        print(f"[robot] command from sketch: {cmd}")
    try:
        _link_sock.sendto(cmd.encode(), ROBOT_LINK)
    except OSError:
        pass


def start_robot_link():
    """Run robot_link.py next to this script; report if macOS blocks Bluetooth."""
    script = Path(__file__).with_name("robot_link.py")
    if not script.exists():
        return None
    proc = subprocess.Popen([sys.executable, "-u", str(script), "--managed"],
                            stdin=subprocess.PIPE)

    def watch():
        code = proc.wait()
        if code != 0:
            print("[robot] Bluetooth link stopped. If macOS blocked Bluetooth for Processing, "
                  "double-click 'Start Robot Link.command' in the fallguard folder.")
    threading.Thread(target=watch, daemon=True).start()
    return proc


def main() -> None:
    ap = argparse.ArgumentParser(description="Stream MediaPipe pose landmarks to Processing.")
    ap.add_argument("--camera", type=int, default=-1,
                    help="camera index; -1 = auto (the Mac's own camera, not the iPhone)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=12000, help="port FallGuard.pde listens on")
    ap.add_argument("--model", choices=["lite", "full", "heavy"], default="full",
                    help="lite = fastest, heavy = most accurate")
    ap.add_argument("--preview", action="store_true", help="show the camera feed with landmarks")
    ap.add_argument("--video-port", type=int, default=12002, help="port for camera JPEGs (0 = off)")
    ap.add_argument("--managed", action="store_true",
                    help="started by FallGuard.pde: quit when the sketch quits")
    args = ap.parse_args()

    if args.managed:
        # The sketch holds our stdin open. When it stops, stdin closes and we quit,
        # so the camera never stays on after you close Processing.
        def watch_parent():
            sys.stdin.read()
            os._exit(0)
        threading.Thread(target=watch_parent, daemon=True).start()
        stop_old_bridges()

    robot_proc = start_robot_link()

    print("Loading pose model...")

    options = vision.PoseLandmarkerOptions(
        base_options=mp_tasks.BaseOptions(model_asset_path=str(ensure_model(args.model))),
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    landmarker = vision.PoseLandmarker.create_from_options(options)

    cam_index = pick_camera() if args.camera < 0 else args.camera
    print(f"Opening camera {cam_index}...")
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        who = "Processing" if args.managed else "your terminal app"
        print(f"Could not open camera {cam_index}. If this keeps happening, allow camera access "
              f"for {who} in System Settings > Privacy & Security > Camera.")
        sys.exit(2)

    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        cmd_sock.bind((args.host, args.port + 1))
        cmd_sock.setblocking(False)
    except OSError:
        print(f"Port {args.port + 1} busy (another bridge running?); robot commands disabled")
        cmd_sock = None

    print("Camera on, looking for you. Step back so your whole body is in frame.")
    t0 = time.monotonic()
    last_ts = -1
    frames, fps_t = 0, time.monotonic()
    misses = 0
    frame_id = 0
    tracker = MarkerTracker()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                misses += 1
                if misses > 200:  # about 2 seconds with no picture
                    print(f"Camera {cam_index} stopped sending pictures. "
                          "Press K in the sketch to try another camera.")
                    sys.exit(2)
                time.sleep(0.01)
                continue
            misses = 0

            frame = cv2.flip(frame, 1)  # mirror, so it behaves like a selfie view
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            ts = int((time.monotonic() - t0) * 1000)
            ts = max(ts, last_ts + 1)  # MediaPipe needs strictly increasing timestamps
            last_ts = ts

            result = landmarker.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts
            )

            if result.pose_landmarks:
                lms = result.pose_landmarks[0]
                msg = "P;" + ";".join(
                    f"{lm.x:.4f},{lm.y:.4f},{(1.0 if lm.visibility is None else lm.visibility):.3f}"
                    for lm in lms
                )
            else:
                lms = None
                msg = "N"
            try:
                out_sock.sendto(msg.encode(), (args.host, args.port))
            except OSError as e:
                print(f"could not send pose: {e}")

            h, w = frame.shape[:2]
            small = cv2.resize(frame, (640, int(640 * h / w)))
            try:
                out_sock.sendto(tracker.find(small).encode(), (args.host, args.port))
            except OSError:
                pass

            if args.video_port:
                ok, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 65])
                if ok:
                    frame_id = (frame_id + 1) & 0xFFFF
                    send_jpeg(out_sock, jpg.tobytes(), frame_id, (args.host, args.video_port))

            # commands coming back from the sketch
            if cmd_sock is not None:
                try:
                    while True:
                        data, _ = cmd_sock.recvfrom(1024)
                        cmd = data.decode().strip()
                        if cmd == "PICK CLEAR B":              # one-marker mode
                            tracker.clear("B")
                        elif cmd.startswith("PICK "):          # "PICK F 0.51 0.63"
                            _, which, nx, ny = cmd.split()
                            tracker.pick(which, float(nx), float(ny), small)
                        else:
                            handle_robot_command(cmd)
                except BlockingIOError:
                    pass

            frames += 1
            if time.monotonic() - fps_t >= 5:
                print(f"tracking at {frames / (time.monotonic() - fps_t):.0f} fps, "
                      f"{'person found' if lms else 'no person in frame'}")
                frames, fps_t = 0, time.monotonic()

            if args.preview:
                if lms:
                    h, w = frame.shape[:2]
                    for lm in lms:
                        cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 4, (0, 255, 120), -1)
                cv2.imshow("pose_bridge preview (q to quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if robot_proc is not None:
            robot_proc.terminate()
        cap.release()
        landmarker.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
