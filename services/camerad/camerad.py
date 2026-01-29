import os
import time
import threading

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)

FPS = int(os.environ.get("CAM_FPS", "30"))
CAP_W = int(os.environ.get("CAM_W", "1920"))
CAP_H = int(os.environ.get("CAM_H", "1080"))
DOWNSAMPLE = int(os.environ.get("CAM_DOWNSAMPLE", "1"))
DEBUG = os.environ.get("CAM_DEBUG", "0") == "1"

STOP = threading.Event()

class Nv12Frame:
    __slots__ = ("pts_ns", "t_wall", "w", "h", "ds", "y", "uv")
    def __init__(self, pts_ns, t_wall, w, h, ds, y, uv):
        self.pts_ns = pts_ns
        self.t_wall = t_wall
        self.w = w
        self.h = h
        self.ds = ds
        self.y = y
        self.uv = uv

class LatestStore:
    def __init__(self):
        self._lock = threading.Lock()
        self.L = None
        self.R = None
        self.lc = 0
        self.rc = 0
        self.dt_ms = None        # latest (L.pts - R.pts) in ms
        self.dt_pts_ns = None    # same delta in ns

    def set_L(self, fr):
        with self._lock:
            self.L = fr
            self.lc += 1
            if self.R is not None:
                self.dt_pts_ns = self.L.pts_ns - self.R.pts_ns
                self.dt_ms = self.dt_pts_ns / 1e6

    def set_R(self, fr):
        with self._lock:
            self.R = fr
            self.rc += 1
            if self.L is not None:
                self.dt_pts_ns = self.L.pts_ns - self.R.pts_ns
                self.dt_ms = self.dt_pts_ns / 1e6

    def get_latest(self):
        # Returns frames + counters + dt in a single lock for consistency
        with self._lock:
            return self.L, self.R, self.lc, self.rc, self.dt_ms

    def get_status(self):
        # Cheap status snapshot (no frame bytes)
        with self._lock:
            have_L = self.L is not None
            have_R = self.R is not None
            return {
                "have_L": have_L,
                "have_R": have_R,
                "lc": self.lc,
                "rc": self.rc,
                "dt_ms": self.dt_ms,
                "w": self.L.w if have_L else None,
                "h": self.L.h if have_L else None,
                "ds": self.L.ds if have_L else None,
            }

store = LatestStore()

def downsample_nv12(y, uv, w, h, ds):
    # pure pixel skipping; stays NV12
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

def make_pipeline(sensor_id):
    # Minimal latency: keep only newest, drop old. NV12 only.
    desc = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={CAP_W},height={CAP_H},format=NV12,framerate={FPS}/1 ! "
        f"nvvidconv ! video/x-raw,format=NV12,width={CAP_W},height={CAP_H} ! "
        f"appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
    )
    return Gst.parse_launch(desc)

def camera_thread(sensor_id, is_left):
    pipeline = make_pipeline(sensor_id)
    sink = pipeline.get_by_name("sink")
    pipeline.set_state(Gst.State.PLAYING)

    if DEBUG:
        print(f"[camerad] cam{sensor_id} started", flush=True)

    try:
        while not STOP.is_set():
            sample = sink.emit("pull-sample")
            if sample is None:
                continue

            buf = sample.get_buffer()
            caps = sample.get_caps()
            s = caps.get_structure(0)
            w = int(s.get_value("width"))
            h = int(s.get_value("height"))
            pts = int(buf.pts) if buf.pts != Gst.CLOCK_TIME_NONE else 0

            ok, mapinfo = buf.map(Gst.MapFlags.READ)
            if not ok:
                continue

            try:
                data = memoryview(mapinfo.data)
                y = data[:w * h]
                uv = data[w * h:w * h + (w * h // 2)]
                yds, uvds, nw, nh = downsample_nv12(y, uv, w, h, DOWNSAMPLE)
                fr = Nv12Frame(pts, time.time(), nw, nh, DOWNSAMPLE, yds, uvds)

                if is_left:
                    store.set_L(fr)
                else:
                    store.set_R(fr)
            finally:
                buf.unmap(mapinfo)
    finally:
        pipeline.set_state(Gst.State.NULL)

def main():
    t0 = threading.Thread(target=camera_thread, args=(0, True))
    t1 = threading.Thread(target=camera_thread, args=(1, False))
    t0.start()
    t1.start()

    if DEBUG:
        print(f"[camerad] LATEST NV12 {CAP_W}x{CAP_H}@{FPS} ds={DOWNSAMPLE}", flush=True)

    try:
        # Quiet keep-alive loop. Other services will read store via IPC later.
        while True:
            if DEBUG:
                st = store.get_status()
                print(f"[camerad] status {st}", flush=True)
            time.sleep(1 if DEBUG else 10)
    except KeyboardInterrupt:
        STOP.set()
        t0.join(timeout=1.0)
        t1.join(timeout=1.0)

if __name__ == "__main__":
    main()
