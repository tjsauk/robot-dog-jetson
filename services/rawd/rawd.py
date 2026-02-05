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

# ---------------------------
# Config
# ---------------------------
HTTP_PORT = int(os.environ.get("RAWD_HTTP_PORT", "8001"))

CAM_W = int(os.environ.get("CAM_W", "1920"))
CAM_H = int(os.environ.get("CAM_H", "1080"))
CAM_MODE = int(os.environ.get("CAM_MODE", "2"))     # 1920x1080@~30 for IMX219
BAYER = os.environ.get("CAM_BAYER", "RGGB")         # from your cam0.txt
BITS = int(os.environ.get("CAM_BITS", "10"))        # from your cam0.txt

# Where nvargus_nvraw writes files (use tmpfs for speed)
TMPDIR = os.environ.get("RAWD_TMPDIR", "/dev/shm")
CAP_SKIPFRAMES = int(os.environ.get("RAWD_SKIPFRAMES", "2"))  # small warm-up
CAP_TIMEOUT_S = float(os.environ.get("RAWD_TIMEOUT_S", "5.0"))

# Preview downsample factor (RAW only, pixel skipping)
PREVIEW_DS = int(os.environ.get("RAWD_PREVIEW_DS", "4"))

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

# ---------------------------
# Storage
# ---------------------------
class LatestRawPair:
    """
    Stores latest RAW Bayer frames for Left (cam0) and Right (cam1),
    plus last UI->Jetson and Teensy->UI telemetry blobs.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self.left = None   # dict
        self.right = None  # dict
        self.ui_in = None      # last UI->Jetson data
        self.teensy_in = None  # last Teensy->Jetson data (to forward to UI)

    def set_frame(self, side, item):
        with self._lock:
            if side == "L":
                self.left = item
            else:
                self.right = item

    def get_pair(self):
        with self._lock:
            L = self.left
            R = self.right
        if not L or not R:
            return None
        dt_ms = (L["t_mono_ns"] - R["t_mono_ns"]) / 1e6
        return L, R, dt_ms

    def set_ui_in(self, payload: bytes):
        with self._lock:
            self.ui_in = {"t_mono_ns": time.monotonic_ns(), "data": payload}

    def get_ui_in(self):
        with self._lock:
            return self.ui_in

    def set_teensy_in(self, payload: bytes):
        with self._lock:
            self.teensy_in = {"t_mono_ns": time.monotonic_ns(), "data": payload}

    def get_teensy_in(self):
        with self._lock:
            return self.teensy_in

store = LatestRawPair()

# ---------------------------
# Capture helpers
# ---------------------------
def _capture_one(sensor_id: int, basepath: str):
    """
    Uses nvargus_nvraw to capture ONE raw frame (plus sidecar .txt).
    Writes basepath.raw and basepath.txt.

    Important: nvargus_nvraw "raw" output includes an extra embedded tail (13056 bytes in your case).
    We will read only the first (W*H*2) bytes as the actual image plane (16-bit container for 10-bit values).
    """
    cmd = [
        "nvargus_nvraw",
        "--c", str(sensor_id),
        "--mode", str(CAM_MODE),
        "--format", "raw",
        "--file", basepath,
        "--skipframes", str(CAP_SKIPFRAMES),
    ]

    # Must run as root (service will run as root)
    p = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=CAP_TIMEOUT_S,
    )
    return p.returncode, p.stdout

def _read_raw_plane(raw_path: str):
    # Actual image plane size (16-bit container)
    img_bytes = CAM_W * CAM_H * 2

    with open(raw_path, "rb") as f:
        data = f.read()

    if len(data) < img_bytes:
        raise RuntimeError(f"RAW file too small: {len(data)} < {img_bytes}")

    plane = data[:img_bytes]  # ignore embedded tail
    return plane

def _downsample_raw16(plane: bytes, ds: int):
    """
    Pixel skipping in RAW16 container (little endian).
    Output is still RAW16 container, but with reduced width/height.
    """
    if ds <= 1:
        return plane, CAM_W, CAM_H

    w, h = CAM_W, CAM_H
    nw, nh = w // ds, h // ds
    out = bytearray(nw * nh * 2)

    # read/write 16-bit words, but keep as bytes
    oi = 0
    row_stride = w * 2
    for r in range(0, h, ds):
        rb = r * row_stride
        for c in range(0, w, ds):
            ib = rb + c * 2
            out[oi:oi+2] = plane[ib:ib+2]
            oi += 2

    return bytes(out), nw, nh

# ---------------------------
# Capture worker (on-demand, but cached)
# ---------------------------
_capture_lock = threading.Lock()

def capture_pair(now: bool = True):
    """
    Captures both cameras (cam0=Left, cam1=Right), updates store.
    Protected with a lock so multiple clients don't collide.
    """
    with _capture_lock:
        t0 = time.monotonic_ns()
        baseL = os.path.join(TMPDIR, "rawd_cam0")
        baseR = os.path.join(TMPDIR, "rawd_cam1")

        rc0, out0 = _capture_one(0, baseL)
        tL = time.monotonic_ns()
        rc1, out1 = _capture_one(1, baseR)
        tR = time.monotonic_ns()

        # If capture failed, raise a useful error
        if rc0 != 0 or rc1 != 0:
            raise RuntimeError(
                "nvargus_nvraw failed:\n"
                f"cam0 rc={rc0}\n{out0}\n"
                f"cam1 rc={rc1}\n{out1}\n"
            )

        rawL = _read_raw_plane(baseL + ".raw")
        rawR = _read_raw_plane(baseR + ".raw")

        store.set_frame("L", {
            "w": CAM_W, "h": CAM_H, "bits": BITS, "bayer": BAYER,
            "t_mono_ns": tL, "t_wall": time.time(),
            "raw16": rawL,
        })
        store.set_frame("R", {
            "w": CAM_W, "h": CAM_H, "bits": BITS, "bayer": BAYER,
            "t_mono_ns": tR, "t_wall": time.time(),
            "raw16": rawR,
        })
        return (time.monotonic_ns() - t0) / 1e9

# ---------------------------
# HTTP API
# ---------------------------
# Snapshot binary format:
# [u32le header_len][header_json][payload]
#
# /snapshot payload: L_raw16 + R_raw16
# /preview payload:  L_raw16_ds + R_raw16_ds   (ds=RAWD_PREVIEW_DS)
#
# header includes sizes, timestamps, dt_ms, and bayer/bits.

class Handler(BaseHTTPRequestHandler):
    def _send_json(self, code: int, obj):
        b = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _read_body(self):
        n = int(self.headers.get("Content-Length", "0"))
        return self.rfile.read(n) if n > 0 else b""

    def do_GET(self):
        u = urlparse(self.path)
        path = u.path
        qs = parse_qs(u.query)

        if path == "/status":
            pair = store.get_pair()
            ui_in = store.get_ui_in()
            teensy_in = store.get_teensy_in()
            out = {
                "ok": True,
                "have_pair": pair is not None,
                "cam": {"w": CAM_W, "h": CAM_H, "mode": CAM_MODE, "bayer": BAYER, "bits": BITS},
                "ui_in": None if not ui_in else {"t_mono_ns": ui_in["t_mono_ns"], "len": len(ui_in["data"])},
                "teensy_in": None if not teensy_in else {"t_mono_ns": teensy_in["t_mono_ns"], "len": len(teensy_in["data"])},
            }
            if pair:
                L, R, dt_ms = pair
                out["dt_ms"] = dt_ms
                out["L"] = {"t_mono_ns": L["t_mono_ns"], "t_wall": L["t_wall"]}
                out["R"] = {"t_mono_ns": R["t_mono_ns"], "t_wall": R["t_wall"]}
            self._send_json(200, out)
            return

        if path == "/capture":
            # Triggers an immediate capture and returns timing
            try:
                sec = capture_pair()
                self._send_json(200, {"ok": True, "capture_s": sec})
            except Exception as e:
                self._send_json(500, {"ok": False, "error": str(e)})
            return

        if path == "/snapshot":
            # Ensure we have fresh frames if requested
            force = qs.get("force", ["0"])[0] == "1"
            if force or store.get_pair() is None:
                try:
                    capture_pair()
                except Exception as e:
                    self._send_json(500, {"ok": False, "error": str(e)})
                    return

            pair = store.get_pair()
            if pair is None:
                self._send_json(503, {"ok": False, "reason": "no_frames"})
                return

            L, R, dt_ms = pair
            header = {
                "format": "BAYER_RAW16_CONTAINER",
                "bayer": BAYER,
                "bits": BITS,
                "mode": CAM_MODE,
                "dt_ms": dt_ms,
                "L": {"w": CAM_W, "h": CAM_H, "t_mono_ns": L["t_mono_ns"], "len": len(L["raw16"])},
                "R": {"w": CAM_W, "h": CAM_H, "t_mono_ns": R["t_mono_ns"], "len": len(R["raw16"])},
            }
            hbytes = json.dumps(header).encode("utf-8")
            payload = L["raw16"] + R["raw16"]
            out = struct.pack("<I", len(hbytes)) + hbytes + payload

            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)
            return

        if path == "/preview":
            # Returns a downsampled RAW pair for cheap UI preview (still Bayer RAW16 container)
            force = qs.get("force", ["0"])[0] == "1"
            if force or store.get_pair() is None:
                try:
                    capture_pair()
                except Exception as e:
                    self._send_json(500, {"ok": False, "error": str(e)})
                    return

            pair = store.get_pair()
            if pair is None:
                self._send_json(503, {"ok": False, "reason": "no_frames"})
                return

            L, R, dt_ms = pair
            Lds, wds, hds = _downsample_raw16(L["raw16"], PREVIEW_DS)
            Rds, _, _ = _downsample_raw16(R["raw16"], PREVIEW_DS)

            header = {
                "format": "BAYER_RAW16_CONTAINER",
                "bayer": BAYER,
                "bits": BITS,
                "mode": CAM_MODE,
                "dt_ms": dt_ms,
                "ds": PREVIEW_DS,
                "L": {"w": wds, "h": hds, "t_mono_ns": L["t_mono_ns"], "len": len(Lds)},
                "R": {"w": wds, "h": hds, "t_mono_ns": R["t_mono_ns"], "len": len(Rds)},
            }
            hbytes = json.dumps(header).encode("utf-8")
            payload = Lds + Rds
            out = struct.pack("<I", len(hbytes)) + hbytes + payload

            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)
            return

        # Telemetry fetch endpoints
        if path == "/telemetry/ui_in":
            item = store.get_ui_in()
            if not item:
                self._send_json(200, {"ok": True, "have": False})
            else:
                # return as base64? nope: return raw bytes in /telemetry/ui_in.bin
                self._send_json(200, {"ok": True, "have": True, "t_mono_ns": item["t_mono_ns"], "len": len(item["data"])})
            return

        if path == "/telemetry/teensy_in":
            item = store.get_teensy_in()
            if not item:
                self._send_json(200, {"ok": True, "have": False})
            else:
                self._send_json(200, {"ok": True, "have": True, "t_mono_ns": item["t_mono_ns"], "len": len(item["data"])})
            return

        # Telemetry raw binary get (so scripts can read exactly what was posted)
        if path == "/telemetry/ui_in.bin":
            item = store.get_ui_in()
            if not item:
                self.send_response(204); self.end_headers(); return
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(item["data"])))
            self.end_headers()
            self.wfile.write(item["data"])
            return

        if path == "/telemetry/teensy_in.bin":
            item = store.get_teensy_in()
            if not item:
                self.send_response(204); self.end_headers(); return
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(item["data"])))
            self.end_headers()
            self.wfile.write(item["data"])
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        u = urlparse(self.path)
        path = u.path
        body = self._read_body()

        # UI -> Jetson commands / telemetry
        if path == "/telemetry/ui_in":
            store.set_ui_in(body)
            self._send_json(200, {"ok": True, "stored": len(body)})
            return

        # Teensy bridge -> Jetson (to forward to UI)
        if path == "/telemetry/teensy_in":
            store.set_teensy_in(body)
            self._send_json(200, {"ok": True, "stored": len(body)})
            return

        self.send_response(404)
        self.end_headers()

def main():
    print(f"[rawd] RAW snapshot server on http://0.0.0.0:{HTTP_PORT}", flush=True)
    print("[rawd] Endpoints:", flush=True)
    print("  GET  /status", flush=True)
    print("  GET  /capture", flush=True)
    print("  GET  /snapshot?force=1", flush=True)
    print("  GET  /preview?force=1", flush=True)
    print("  POST /telemetry/ui_in    (UI->Jetson bytes)", flush=True)
    print("  POST /telemetry/teensy_in (Teensy->UI bytes)", flush=True)

    try:
        httpd = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    except Exception as e:
        print(f"[rawd] FATAL: cannot bind HTTP server on port {HTTP_PORT}: {e}", flush=True)
        raise

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            httpd.server_close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
