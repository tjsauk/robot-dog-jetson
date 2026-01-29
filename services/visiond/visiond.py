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

# Localhost UDP ports for RTP/H264 (internal only)
UDP_PORT_L = int(os.environ.get("UDP_PORT_L", "5000"))
UDP_PORT_R = int(os.environ.get("UDP_PORT_R", "5002"))

STOP = threading.Event()


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

    # Y plane (full res): pick every ds pixel
    y_out = bytearray(nw * nh)
    oi = 0
    for r in range(0, h, ds):
        base = r * w
        for c in range(0, w, ds):
            y_out[oi] = y[base + c]
            oi += 1

    # UV plane (half vertical res, interleaved U,V per 2 pixels)
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
        step = ds * 2  # U,V stride (2 bytes per 2 pixels)
        for ic in range(0, w, step):
            if oc + 1 >= nw:
                break
            uv_out[ob + oc] = uv[ib + ic]
            uv_out[ob + oc + 1] = uv[ib + ic + 1]
            oc += 2
        orow += 1

    return bytes(y_out), bytes(uv_out), nw, nh


# --- Capture pipeline (single pipeline per sensor) ---
# nvargus -> tee
#   branch A: nvvidconv -> NV12 (CPU) -> appsink (snapshots)
#   branch B: HW H264 -> RTP -> udpsink to localhost (for RTSP restream)
def start_capture_pipeline(sensor_id: int, side_tag: str, udp_port: int):
    # NOTE: appsink must be CPU memory to map in python => nvvidconv to system memory
    # Encoder branch stays in NVMM for speed.
    appsink_name = f"appsink_{side_tag}"

    # queue leaky=downstream ensures nothing blocks; max-buffers=1 => latest only.
    launch = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={W},height={H},format=NV12,framerate={FPS}/1 ! "
        f"tee name=t "
        f"t. ! queue leaky=downstream max-size-buffers=1 ! "
        f"nvvidconv ! video/x-raw,format=NV12,width={W},height={H} ! "
        f"appsink name={appsink_name} emit-signals=true sync=false max-buffers=1 drop=true "
        f"t. ! queue leaky=downstream max-size-buffers=4 ! "
        f"nvv4l2h264enc bitrate={BITRATE} insert-sps-pps=true iframeinterval={FPS} ! "
        f"h264parse ! rtph264pay pt=96 config-interval=1 ! "
        f"udpsink host=127.0.0.1 port={udp_port} sync=false async=false"
    )

    pipeline = Gst.parse_launch(launch)
    appsink = pipeline.get_by_name(appsink_name)
    if appsink is None:
        raise RuntimeError(f"appsink {appsink_name} not found in pipeline")

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

    pipeline.set_state(Gst.State.PLAYING)

    if DEBUG:
        print(f"[visiond] capture pipeline started side={side_tag} sensor={sensor_id} udp={udp_port}", flush=True)

    return pipeline


# --- RTSP factories: restream from localhost UDP RTP/H264 ---
def make_udp_rtsp_factory(udp_port: int):
    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_shared(True)

    # The caps MUST match what rtph264pay sends: RTP media=video encoding-name=H264 payload=96
    launch = (
        f"( udpsrc port={udp_port} caps="
        f"\"application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000\" "
        f"! rtph264depay ! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )
    factory.set_launch(launch)
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
                "L": {
                    "w": L["w"], "h": L["h"], "ds": L["ds"], "pts_ns": L["pts_ns"],
                    "y_len": len(L["y"]), "uv_len": len(L["uv"]),
                },
                "R": {
                    "w": R["w"], "h": R["h"], "ds": R["ds"], "pts_ns": R["pts_ns"],
                    "y_len": len(R["y"]), "uv_len": len(R["uv"]),
                },
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

    # quiet down the default logging
    def log_message(self, format, *args):
        if DEBUG:
            super().log_message(format, *args)


def http_server_thread():
    server = HTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[visiond] HTTP snapshot API on http://0.0.0.0:{HTTP_PORT}", flush=True)
    server.serve_forever()


def main():
    # Start HTTP server in background
    threading.Thread(target=http_server_thread, daemon=True).start()

    # Start persistent capture pipelines (one per sensor)
    # This ensures /snapshot works even with no RTSP clients.
    pipelines = []
    try:
        pipelines.append(start_capture_pipeline(0, "L", UDP_PORT_L))
        pipelines.append(start_capture_pipeline(1, "R", UDP_PORT_R))
    except Exception as e:
        print(f"[visiond] ERROR starting capture pipelines: {e}", flush=True)
        raise

    # Start RTSP server (restream from localhost UDP)
    rtsp = GstRtspServer.RTSPServer()
    rtsp.set_address("0.0.0.0")
    rtsp.set_service(str(RTSP_PORT))
    mounts = rtsp.get_mount_points()

    mounts.add_factory("/cam0", make_udp_rtsp_factory(UDP_PORT_L))
    mounts.add_factory("/cam1", make_udp_rtsp_factory(UDP_PORT_R))

    attach_id = rtsp.attach(None)
    print(f"[visiond] rtsp.attach id: {attach_id}", flush=True)

    print("[visiond] RTSP ready:", flush=True)
    print(f"[visiond]   rtsp://<JETSON_IP>:{RTSP_PORT}/cam0  (Left)", flush=True)
    print(f"[visiond]   rtsp://<JETSON_IP>:{RTSP_PORT}/cam1  (Right)", flush=True)

    # Run GLib main loop (must be main thread)
    GLib.MainLoop().run()

    # Cleanup (normally unreachable)
    for p in pipelines:
        p.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
