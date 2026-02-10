#!/usr/bin/env python3
import os
import time
import json
import struct
import threading
import subprocess
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import urlparse, parse_qs

import numpy as np

# ---------------------------
# Config (lock to your proven-good behavior)
# ---------------------------
HTTP_PORT = int(os.environ.get("RAWD_HTTP_PORT", "8001"))

# Capture resolution (raw Bayer). For IMX219 full-res mode 0 is typically 3264x2464.
CAP_W = int(os.environ.get("CAP_W", "3264"))
CAP_H = int(os.environ.get("CAP_H", "2464"))
CAP_MODE = int(os.environ.get("CAP_MODE", "0"))         # you used --mode 0 successfully
CAP_SKIPFRAMES = int(os.environ.get("RAWD_SKIPFRAMES", "2"))
CAP_TIMEOUT_S = float(os.environ.get("RAWD_TIMEOUT_S", "6.0"))
TARGET_FPS = float(os.environ.get("RAWD_FPS", "30.0"))

TMPDIR = os.environ.get("RAWD_TMPDIR", "/dev/shm")

# Output sizes after rgb2x2 + preview subsample
RGB_W = CAP_W // 2         # 1632
RGB_H = CAP_H // 2         # 1232
PREVIEW_W = RGB_W // 2     # 816
PREVIEW_H = RGB_H // 2     # 616

# Bayer format is assumed RGGB for the rgb2x2 block mapping.
BAYER = os.environ.get("CAM_BAYER", "RGGB")
BITS = int(os.environ.get("CAM_BITS", "10"))

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

def monotonic_ns():
    if hasattr(time, "monotonic_ns"):
        return time.monotonic_ns()
    return int(time.monotonic() * 1e9)

# ---------------------------
# Storage
# ---------------------------
class LatestRGBPair:
    """
    Stores latest processed RGB2x2 frames for Left (cam0) and Right (cam1),
    plus preview frames, plus timing stats.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self.left = None   # dict
        self.right = None  # dict
        self.stats = {
            "last_loop_ns": None,
            "cam0_dt_ms": None,
            "cam1_dt_ms": None,
            "lr_dt_ms": None,
            "fps_est": None,
        }

    def set_pair(self, L, R, stats_update):
        with self._lock:
            self.left = L
            self.right = R
            self.stats.update(stats_update)

    def get_pair(self):
        with self._lock:
            return self.left, self.right, dict(self.stats)

store = LatestRGBPair()

# ---------------------------
# Capture + decode helpers
# ---------------------------
def _capture_one(sensor_id: int, basepath: str):
    cmd = [
        "nvargus_nvraw",
        "--c", str(sensor_id),
        "--mode", str(CAP_MODE),
        "--format", "raw",
        "--file", basepath,
        "--skipframes", str(CAP_SKIPFRAMES),
    ]
    p = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        timeout=CAP_TIMEOUT_S,
    )
    return p.returncode, p.stdout

def _read_raw_plane_bytes(raw_path: str) -> bytes:
    img_bytes = CAP_W * CAP_H * 2
    with open(raw_path, "rb") as f:
        data = f.read()
    if len(data) < img_bytes:
        raise RuntimeError(f"RAW too small: {len(data)} < {img_bytes}")
    # Ignore any tail/metadata by taking only the image plane
    return data[:img_bytes]

def _rawplane_to_rgb2x2_rot180(plane_bytes: bytes) -> np.ndarray:
    """
    plane_bytes: little-endian u16 container, shape (CAP_H, CAP_W)
    unpack: raw10 = (u16 >> 2) & 0x03FF
    map RGGB 2x2 to RGB:
      R = (0,0), G = avg((0,1),(1,0)), B = (1,1)
    output: uint8 RGB, shape (RGB_H, RGB_W, 3)
    rotate 180 degrees.
    """
    u16 = np.frombuffer(plane_bytes, dtype="<u2", count=CAP_W*CAP_H).reshape(CAP_H, CAP_W)
    raw10 = ((u16 >> 2) & 0x03FF).astype(np.uint16)

    # RGGB blocks
    R = raw10[0::2, 0::2]
    G1 = raw10[0::2, 1::2]
    G2 = raw10[1::2, 0::2]
    B = raw10[1::2, 1::2]
    G = ((G1.astype(np.uint32) + G2.astype(np.uint32)) // 2).astype(np.uint16)

    rgb16 = np.dstack([R, G, B]).astype(np.uint16)   # RGB order
    rgb8 = (rgb16 >> 2).astype(np.uint8)            # 10->8

    # rotate 180: flip both axes
    rgb8 = rgb8[::-1, ::-1, :]
    return rgb8

def _preview_from_rgb2x2(rgb8: np.ndarray) -> np.ndarray:
    # subsample every 2 pixels in both directions -> 816x616
    return rgb8[0::2, 0::2, :].copy()

# ---------------------------
# Capture loop thread
# ---------------------------
_stop_evt = threading.Event()

def capture_loop():
    os.makedirs(TMPDIR, exist_ok=True)
    baseL = os.path.join(TMPDIR, "rawd_cam0")
    baseR = os.path.join(TMPDIR, "rawd_cam1")

    last_cam0_ns = None
    last_cam1_ns = None
    last_loop_ns = None

    target_period_ns = int(1e9 / TARGET_FPS) if TARGET_FPS > 0 else 0

    while not _stop_evt.is_set():
        try:
            loop_start = monotonic_ns()

            # Capture both cameras (sequential; simple + reliable)
            rc0, out0 = _capture_one(0, baseL)
            t0 = monotonic_ns()
            rc1, out1 = _capture_one(1, baseR)
            t1 = monotonic_ns()

            if rc0 != 0 or rc1 != 0:
                # Don’t crash the daemon—store error in stats and continue
                err = f"cam0 rc={rc0} cam1 rc={rc1}"
                print("[rawd] capture error:", err, flush=True)
                print("[rawd] cam0 output:\n", out0, flush=True)
                print("[rawd] cam1 output:\n", out1, flush=True)
                # small backoff
                time.sleep(0.2)
                continue

            try:
                rawL = _read_raw_plane_bytes(baseL + ".raw")
                rawR = _read_raw_plane_bytes(baseR + ".raw")
                rgbL = _rawplane_to_rgb2x2_rot180(rawL)
                rgbR = _rawplane_to_rgb2x2_rot180(rawR)
                prevL = _preview_from_rgb2x2(rgbL)
                prevR = _preview_from_rgb2x2(rgbR)
            except Exception as e:
                print("[rawd] decode error:", e, flush=True)
                time.sleep(0.1)
                continue

            # Timing stats
            cam0_dt_ms = None
            cam1_dt_ms = None
            if last_cam0_ns is not None:
                cam0_dt_ms = (t0 - last_cam0_ns) / 1e6
            if last_cam1_ns is not None:
                cam1_dt_ms = (t1 - last_cam1_ns) / 1e6

            lr_dt_ms = (t1 - t0) / 1e6

            fps_est = None
            if last_loop_ns is not None:
                loop_dt_s = (loop_start - last_loop_ns) / 1e9
                if loop_dt_s > 0:
                    fps_est = 1.0 / loop_dt_s

            last_cam0_ns = t0
            last_cam1_ns = t1
            last_loop_ns = loop_start

            L = {
                "cam": 0,
                "t_mono_ns": t0,
                "t_wall": time.time(),
                "w": RGB_W, "h": RGB_H,
                "rgb": rgbL,          # numpy uint8
                "preview": prevL,     # numpy uint8
            }
            R = {
                "cam": 1,
                "t_mono_ns": t1,
                "t_wall": time.time(),
                "w": RGB_W, "h": RGB_H,
                "rgb": rgbR,
                "preview": prevR,
            }

            store.set_pair(L, R, {
                "last_loop_ns": loop_start,
                "cam0_dt_ms": cam0_dt_ms,
                "cam1_dt_ms": cam1_dt_ms,
                "lr_dt_ms": lr_dt_ms,
                "fps_est": fps_est,
            })

            # Sleep to target FPS if we’re faster than target (usually we won’t be)
            if target_period_ns > 0:
                elapsed_ns = monotonic_ns() - loop_start
                remain_ns = target_period_ns - elapsed_ns
                if remain_ns > 0:
                    time.sleep(remain_ns / 1e9)
        except Exception as e:
            import traceback
            print("[rawd] FATAL capture_loop exception:", e, flush=True)
            traceback.print_exc()
            time.sleep(0.5)
            continue

# ---------------------------
# HTTP API (binary packets)
# ---------------------------
# Binary format:
# [u32le header_len][header_json_utf8][payload]
# payload for /rgb:     L_rgb + R_rgb (uint8, RGB interleaved)
# payload for /preview: L_prev + R_prev

def pack_frame_pair(L, R, kind: str):
    if kind == "rgb":
        A = L["rgb"]
        B = R["rgb"]
        w, h = RGB_W, RGB_H
    elif kind == "preview":
        A = L["preview"]
        B = R["preview"]
        w, h = PREVIEW_W, PREVIEW_H
    else:
        raise ValueError("bad kind")

    payloadA = A.tobytes(order="C")
    payloadB = B.tobytes(order="C")
    payload = payloadA + payloadB

    header = {
        "format": "RGB_U8_INTERLEAVED",
        "kind": kind,
        "bayer_assumed": BAYER,
        "bits": BITS,
        "capture": {"w": CAP_W, "h": CAP_H, "mode": CAP_MODE},
        "out": {"w": w, "h": h, "channels": 3},
        "L": {"t_mono_ns": L["t_mono_ns"], "t_wall": L["t_wall"], "len": len(payloadA)},
        "R": {"t_mono_ns": R["t_mono_ns"], "t_wall": R["t_wall"], "len": len(payloadB)},
    }
    hbytes = json.dumps(header).encode("utf-8")
    return struct.pack("<I", len(hbytes)) + hbytes + payload

class Handler(BaseHTTPRequestHandler):
    def _send_json(self, code: int, obj):
        b = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        path = u.path

        if path == "/status":
            L, R, st = store.get_pair()
            out = {
                "ok": True,
                "have_pair": bool(L and R),
                "capture": {"w": CAP_W, "h": CAP_H, "mode": CAP_MODE},
                "out": {"rgb_w": RGB_W, "rgb_h": RGB_H, "prev_w": PREVIEW_W, "prev_h": PREVIEW_H},
                "stats": st,
            }
            if L and R:
                out["L"] = {"t_mono_ns": L["t_mono_ns"], "t_wall": L["t_wall"]}
                out["R"] = {"t_mono_ns": R["t_mono_ns"], "t_wall": R["t_wall"]}
            self._send_json(200, out)
            return

        if path == "/rgb":
            L, R, _ = store.get_pair()
            if not L or not R:
                self._send_json(503, {"ok": False, "reason": "no_frames_yet"})
                return
            out = pack_frame_pair(L, R, "rgb")
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)
            return

        if path == "/preview":
            L, R, _ = store.get_pair()
            if not L or not R:
                self._send_json(503, {"ok": False, "reason": "no_frames_yet"})
                return
            out = pack_frame_pair(L, R, "preview")
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)
            return

        self.send_response(404)
        self.end_headers()

def main():
    print(f"[rawd] starting capture loop (target_fps={TARGET_FPS})", flush=True)
    th = threading.Thread(target=capture_loop, daemon=True)
    th.start()

    print(f"[rawd] HTTP on 0.0.0.0:{HTTP_PORT}", flush=True)
    print("[rawd] GET /status, /rgb, /preview", flush=True)

    httpd = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        _stop_evt.set()
        try:
            httpd.server_close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
