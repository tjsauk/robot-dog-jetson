#!/usr/bin/env python3
import os
import time
import json
import struct
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib

Gst.init(None)

# --- Config ---
RTSP_PORT = int(os.environ.get("RTSP_PORT", "8554"))
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8000"))

W = int(os.environ.get("CAM_W", "1920"))
H = int(os.environ.get("CAM_H", "1080"))
FPS = int(os.environ.get("CAM_FPS", "30"))
BITRATE = int(os.environ.get("BITRATE", "8000000"))  # bits/s

DOWNSAMPLE = int(os.environ.get("CAM_DOWNSAMPLE", "1"))  # 1 = no downsample
DEBUG = os.environ.get("VISION_DEBUG", "0") == "1"

# --- Latest frame store (NV12 split into Y and UV planes) ---
class NV12Latest:
    def __init__(self):
        self.lock = threading.Lock()
        self.left = None   # dict
        self.right = None  # dict

    def update(self, side, pts_ns, w, h, ds, y_bytes, uv_bytes):
        item = {
            "pts_ns": int(pts_ns),
            "t_wall": time.time(),
            "w": int(w),
            "h": int(h),
            "ds": int(ds),
            "y": y_bytes,
            "uv": uv_bytes,
        }
        with self.lock:
            if side == "L":
                self.left = item
            else:
                self.right = item

    def snapshot(self):
        with self.lock:
            L = self.left
            R = self.right
        if (L is None) or (R is None):
            return None
        dt_ms = (L["pts_ns"] - R["pts_ns"]) / 1e6
        return L, R, dt_ms

store = NV12Latest()

def downsample_nv12(y, uv, w, h, ds):
    # pure pixel skipping, keeps NV12 layout
    if ds <= 1:
        return bytes(y), bytes(uv), w, h

    nw = w // ds
    nh = h // ds

    y_out = bytearray(nw * nh)
    oi = 0
    for r in range(0, h, ds):
        base = r * w
        for c in range(0, w, ds):
            y_out[oi] = y[base + c]
            oi += 1

    uv_h = h // 2
    uv_out_h = nh // 2
    uv_out = bytearray(nw * uv_out_h)

    orow = 0
    for ur in range(0, uv_h, ds):
        if orow >= uv_out_h:
            break
        ib = ur * w
        ob = orow * nw
        oc = 0
        step = ds * 2
        for ic in range(0, w, step):
            if oc + 1 >= nw:
                break
            uv_out[ob + oc] = uv[ib + ic]
            uv_out[ob + oc + 1] = uv[ib + ic + 1]
            oc += 2
        orow += 1

    return bytes(y_out), bytes(uv_out), nw, nh

# --- RTSP factory with tee: appsink for NV12 + pay0 for RTSP ---
def make_factory(sensor_id, side_tag):
    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_shared(True)

    launch = (
        f"( nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={W},height={H},format=NV12,framerate={FPS}/1 ! "
        f"nvvidconv ! video/x-raw,format=NV12,width={W},height={H} ! "
        f"tee name=t "
        f"t. ! queue leaky=downstream max-size-buffers=1 ! "
        f"appsink name=appsink_{side_tag} emit-signals=true sync=false max-buffers=1 drop=true "
        f"t. ! queue leaky=downstream max-size-buffers=4 ! "
        f"nvv4l2h264enc bitrate={BITRATE} insert-sps-pps=true iframeinterval={FPS} ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )
    factory.set_launch(launch)

    def on_media_configure(factory, media):
        # Connect appsink callback to capture NV12 frames into store
        element = media.get_element()
        appsink = element.get_by_name(f"appsink_{side_tag}")
        if appsink is None:
            print(f"[visiond] ERROR: appsink_{side_tag} not found", flush=True)
            return

        def on_new_sample(sink):
            sample = sink.emit("pull-sample")
            if sample is None:
                return Gst.FlowReturn.OK

            buf = sample.get_buffer()
            caps = sample.get_caps()
            s = caps.get_structure(0)
            w = int(s.get_value("width"))
            h = int(s.get_value("height"))
            pts = int(buf.pts) if buf.pts != Gst.CLOCK_TIME_NONE else 0

            ok, mapinfo = buf.map(Gst.MapFlags.READ)
            if not ok:
                return Gst.FlowReturn.OK

            try:
                data = memoryview(mapinfo.data)
                y = data[:w * h]
                uv = data[w * h:w * h + (w * h // 2)]
                yds, uvds, nw, nh = downsample_nv12(y, uv, w, h, DOWNSAMPLE)
                store.update(side_tag, pts, nw, nh, DOWNSAMPLE, yds, uvds)
            finally:
                buf.unmap(mapinfo)

            return Gst.FlowReturn.OK

        appsink.connect("new-sample", on_new_sample)

    factory.connect("media-configure", on_media_configure)
    return factory

# --- HTTP snapshot API ---
# Binary format:
# [u32 little endian header_len][header_json_bytes][payload bytes]
# payload = L_y + L_uv + R_y + R_uv
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/status"):
            snap = store.snapshot()
            if snap is None:
                out = {"ok": False, "reason": "no_frames_yet"}
            else:
                L, R, dt_ms = snap
                out = {
                    "ok": True,
                    "dt_ms": dt_ms,
                    "L": {"w": L["w"], "h": L["h"], "ds": L["ds"], "pts_ns": L["pts_ns"]},
                    "R": {"w": R["w"], "h": R["h"], "ds": R["ds"], "pts_ns": R["pts_ns"]},
                }
            b = json.dumps(out).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
            return

        if self.path.startswith("/snapshot"):
            snap = store.snapshot()
            if snap is None:
                self.send_response(503)
                self.end_headers()
                return

            L, R, dt_ms = snap
            header = {
                "format": "NV12",
                "dt_ms": dt_ms,
                "L": {"w": L["w"], "h": L["h"], "ds": L["ds"], "pts_ns": L["pts_ns"], "y_len": len(L["y"]), "uv_len": len(L["uv"])},
                "R": {"w": R["w"], "h": R["h"], "ds": R["ds"], "pts_ns": R["pts_ns"], "y_len": len(R["y"]), "uv_len": len(R["uv"])},
            }
            hbytes = json.dumps(header).encode("utf-8")
            payload = L["y"] + L["uv"] + R["y"] + R["uv"]

            out = struct.pack("<I", len(hbytes)) + hbytes + payload

            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)
            return

        self.send_response(404)
        self.end_headers()

def http_server_thread():
    server = HTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[visiond] HTTP snapshot API on http://0.0.0.0:{HTTP_PORT}", flush=True)
    server.serve_forever()

def main():
    # Start HTTP server
    threading.Thread(target=http_server_thread, daemon=True).start()

    # Start RTSP server
    rtsp = GstRtspServer.RTSPServer()
    rtsp.set_service(str(RTSP_PORT))
    mounts = rtsp.get_mount_points()

    mounts.add_factory("/cam0", make_factory(0, "L"))
    mounts.add_factory("/cam1", make_factory(1, "R"))

    rtsp.attach(None)

    print("[visiond] RTSP ready:", flush=True)
    print(f"[visiond]   rtsp://192.168.50.79:{RTSP_PORT}/cam0  (Left)", flush=True)
    print(f"[visiond]   rtsp://192.168.50.79:{RTSP_PORT}/cam1  (Right)", flush=True)
    if DEBUG:
        print("[visiond] DEBUG enabled", flush=True)

    GLib.MainLoop().run()

if __name__ == "__main__":
    main()
