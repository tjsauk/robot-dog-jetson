import os
import time
import threading
from collections import deque

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)

FPS = int(os.environ.get("CAM_FPS", "30"))
CAP_W = int(os.environ.get("CAM_W", "1920"))
CAP_H = int(os.environ.get("CAM_H", "1080"))
DOWNSAMPLE = int(os.environ.get("CAM_DOWNSAMPLE", "2"))
SYNC_TOL_MS = float(os.environ.get("CAM_SYNC_TOL_MS", "10.0"))
RING = int(os.environ.get("CAM_RING", "12"))  # frames kept per camera

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

def downsample_nv12(y, uv, w, h, ds):
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

class RingStore:
    def __init__(self, maxlen):
        self._lock = threading.Lock()
        self.L = deque(maxlen=maxlen)
        self.R = deque(maxlen=maxlen)
        self.lc = 0
        self.rc = 0

    def push_L(self, fr):
        with self._lock:
            self.L.append(fr)
            self.lc += 1

    def push_R(self, fr):
        with self._lock:
            self.R.append(fr)
            self.rc += 1

    def best_pair(self):
        """Return (Lframe, Rframe, dt_ms, lc, rc) using closest pts."""
        with self._lock:
            if not self.L or not self.R:
                return None

            # Copy to avoid holding lock while searching
            L = list(self.L)
            R = list(self.R)
            lc, rc = self.lc, self.rc

        best = None
        best_abs = None

        # Two-pointer style search could be done if sorted; deques are in time order.
        j = 0
        for lf in L:
            while j + 1 < len(R) and abs((lf.pts_ns - R[j+1].pts_ns)) <= abs((lf.pts_ns - R[j].pts_ns)):
                j += 1
            dt_ns = lf.pts_ns - R[j].pts_ns
            a = abs(dt_ns)
            if best_abs is None or a < best_abs:
                best_abs = a
                best = (lf, R[j], dt_ns / 1e6, lc, rc)

        return best

store = RingStore(RING)

def make_pipeline(sensor_id):
    # NV12 direct; appsink in system memory (minimal necessary to access bytes)
    desc = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={CAP_W},height={CAP_H},format=NV12,framerate={FPS}/1 ! "
        f"nvvidconv ! video/x-raw,width={CAP_W},height={CAP_H},format=NV12 ! "
        f"appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
    )
    return Gst.parse_launch(desc)

def camera_thread(sensor_id, is_left):
    pipeline = make_pipeline(sensor_id)
    sink = pipeline.get_by_name("sink")
    pipeline.set_state(Gst.State.PLAYING)

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
                    store.push_L(fr)
                else:
                    store.push_R(fr)
            finally:
                buf.unmap(mapinfo)

    finally:
        pipeline.set_state(Gst.State.NULL)

def main():
    threading.Thread(target=camera_thread, args=(0, True), daemon=True).start()
    threading.Thread(target=camera_thread, args=(1, False), daemon=True).start()

    print(f"[camerad] NV12 ring-sync. {CAP_W}x{CAP_H}@{FPS} ds={DOWNSAMPLE} ring={RING} tol={SYNC_TOL_MS}ms", flush=True)

    last_lc = last_rc = 0
    while True:
        best = store.best_pair()
        if best is None:
            print("[camerad] waiting for cameras...", flush=True)
            time.sleep(1)
            continue

        lf, rf, dt_ms, lc, rc = best
        dl = lc - last_lc
        dr = rc - last_rc
        last_lc, last_rc = lc, rc

        synced = abs(dt_ms) <= SYNC_TOL_MS
        print(f"[camerad] +L{dl} +R{dr} pts_dt={dt_ms:.2f}ms synced={synced}", flush=True)

        time.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        STOP.set()
        time.sleep(0.5)