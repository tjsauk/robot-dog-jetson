import os
import time
import threading
from typing import Optional, Tuple

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)

FPS = int(os.environ.get("CAM_FPS", "30"))
CAP_W = int(os.environ.get("CAM_W", "1920"))
CAP_H = int(os.environ.get("CAM_H", "1080"))
DOWNSAMPLE = int(os.environ.get("CAM_DOWNSAMPLE", "1"))
SYNC_TOL_MS = float(os.environ.get("CAM_SYNC_TOL_MS", "10.0"))

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
        self.left = None
        self.right = None
        self.left_count = 0
        self.right_count = 0

    def set_left(self, fr):
        with self._lock:
            self.left = fr
            self.left_count += 1

    def set_right(self, fr):
        with self._lock:
            self.right = fr
            self.right_count += 1

    def get_pair(self):
        with self._lock:
            if self.left is None or self.right is None:
                return None
            return self.left, self.right, self.left_count, self.right_count


store = LatestStore()


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


def make_pipeline(sensor_id):
    desc = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={CAP_W},height={CAP_H},format=NV12,framerate={FPS}/1 ! "
        f"nvvidconv ! video/x-raw,width={CAP_W},height={CAP_H},format=NV12 ! "
        f"appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
    )
    return Gst.parse_launch(desc)


def run_camera(sensor_id, is_left):
    pipeline = make_pipeline(sensor_id)
    sink = pipeline.get_by_name("sink")
    pipeline.set_state(Gst.State.PLAYING)

    while True:
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
                store.set_left(fr)
            else:
                store.set_right(fr)

        finally:
            buf.unmap(mapinfo)


def main():
    threading.Thread(target=run_camera, args=(0, True), daemon=True).start()
    threading.Thread(target=run_camera, args=(1, False), daemon=True).start()

    print(f"[camerad] NV12 direct {CAP_W}x{CAP_H} ds={DOWNSAMPLE}", flush=True)

    ll = lr = 0
    while True:
        snap = store.get_pair()
        if snap:
            l, r, lc, rc = snap
            print(
                f"[camerad] +L{lc-ll} +R{rc-lr} pts_dt={(l.pts_ns-r.pts_ns)/1e6:.2f}ms",
                flush=True,
            )
            ll, lr = lc, rc
        else:
            print("[camerad] waiting for cameras...", flush=True)
        time.sleep(2)


if __name__ == "__main__":
    main()