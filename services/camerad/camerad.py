import os
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

Gst.init(None)

FPS = int(os.environ.get("CAM_FPS", "30"))

# These are the capture dimensions from Argus (choose a sensor mode you want)
CAP_W = int(os.environ.get("CAM_W", "1920"))
CAP_H = int(os.environ.get("CAM_H", "1080"))

# Downsample by integer skipping:
# 1 = no downsample, 2 = take every 2nd pixel, etc.
DOWNSAMPLE = int(os.environ.get("CAM_DOWNSAMPLE", "1"))

# How close in time we require L/R frames to be considered "synced"
SYNC_TOL_MS = float(os.environ.get("CAM_SYNC_TOL_MS", "10.0"))


@dataclass
class Nv12Frame:
    pts_ns: int          # GStreamer PTS in nanoseconds
    t_wall: float        # wall clock time when received
    w: int
    h: int
    ds: int              # downsample factor applied
    y: bytes             # luma plane bytes (downsampled if ds>1)
    uv: bytes            # interleaved chroma bytes (downsampled if ds>1)


class LatestStore:
    def __init__(self):
        self._lock = threading.Lock()
        self.left: Optional[Nv12Frame] = None
        self.right: Optional[Nv12Frame] = None
        self.left_count = 0
        self.right_count = 0

    def set_left(self, fr: Nv12Frame):
        with self._lock:
            self.left = fr
            self.left_count += 1

    def set_right(self, fr: Nv12Frame):
        with self._lock:
            self.right = fr
            self.right_count += 1

    def get_pair_best_effort(self) -> Optional[Tuple[Nv12Frame, Nv12Frame, int, int]]:
        with self._lock:
            if self.left is None or self.right is None:
                return None
            return self.left, self.right, self.left_count, self.right_count


store = LatestStore()


def downsample_nv12(y: memoryview, uv: memoryview, w: int, h: int, ds: int) -> Tuple[bytes, bytes, int, int]:
    """
    NV12: Y plane is w*h bytes.
         UV plane is (w*h)/2 bytes, interleaved U,V at half vertical resolution.
    Downsample by skipping pixels:
      - Y: take every ds pixel in x and y
      - UV: must match 2x2 luma blocks, so we downsample UV consistently.
    """
    if ds <= 1:
        return bytes(y), bytes(uv), w, h

    # New dims
    nw = w // ds
    nh = h // ds

    # Y plane downsample: pick every ds-th pixel and ds-th row
    y_out = bytearray(nw * nh)
    out_i = 0
    for row in range(0, h, ds):
        base = row * w
        for col in range(0, w, ds):
            y_out[out_i] = y[base + col]
            out_i += 1

    # UV plane is half height: h/2 rows, each row w bytes (U,V interleaved)
    # For NV12, each 2x2 luma block corresponds to one UV sample pair.
    # When downsampling by ds, we should sample UV at step ds in x (but in units of 2 bytes),
    # and step ds in y (but UV rows correspond to 2 luma rows).
    uv_h = h // 2
    uv_out_h = nh // 2
    uv_out_w_bytes = nw  # NV12 UV row has same byte width as Y row (because interleaved)

    uv_out = bytearray(uv_out_w_bytes * uv_out_h)

    out_row = 0
    for uv_row in range(0, uv_h, ds):
        if out_row >= uv_out_h:
            break
        in_base = uv_row * w
        out_base = out_row * uv_out_w_bytes

        out_col = 0
        # step in UV must keep U,V pairs aligned -> increment by ds*2 bytes
        step = ds * 2
        for in_col in range(0, w, step):
            if out_col + 1 >= uv_out_w_bytes:
                break
            uv_out[out_base + out_col] = uv[in_base + in_col]       # U
            uv_out[out_base + out_col + 1] = uv[in_base + in_col+1] # V
            out_col += 2

        out_row += 1

    return bytes(y_out), bytes(uv_out), nw, nh


def make_pipeline(sensor_id: int) -> Gst.Pipeline:
    # IMPORTANT: no videoconvert, no BGR. We keep NV12.
    # appsink emits CPU-visible buffers, so NVMM -> system memory occurs (necessary to access bytes).
    desc = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={CAP_W},height={CAP_H},format=NV12,framerate={FPS}/1 ! "
        f"nvvidconv ! video/x-raw,width={CAP_W},height={CAP_H},format=NV12 ! "
        f"appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
    )
    pipeline = Gst.parse_launch(desc)
    assert isinstance(pipeline, Gst.Pipeline)
    return pipeline


def run_camera(sensor_id: int, is_left: bool):
    pipeline = make_pipeline(sensor_id)
    sink = pipeline.get_by_name("sink")
    if sink is None:
        raise RuntimeError("appsink not found in pipeline")

    pipeline.set_state(Gst.State.PLAYING)

    try:
        while True:
            sample = sink.emit("pull-sample")
            if sample is None:
                time.sleep(0.001)
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
                y_size = w * h
                uv_size = (w * h) // 2

                y = data[:y_size]
                uv = data[y_size:y_size + uv_size]

                y_ds, uv_ds, nw, nh = downsample_nv12(y, uv, w, h, DOWNSAMPLE)

                fr = Nv12Frame(
                    pts_ns=pts,
                    t_wall=time.time(),
                    w=nw,
                    h=nh,
                    ds=DOWNSAMPLE,
                    y=y_ds,
                    uv=uv_ds,
                )

                if is_left:
                    store.set_left(fr)
                else:
                    store.set_right(fr)

            finally:
                buf.unmap(mapinfo)

    finally:
        pipeline.set_state(Gst.State.NULL)


def pts_delta_ms(a: Nv12Frame, b: Nv12Frame) -> float:
    return (a.pts_ns - b.pts_ns) / 1e6


def main():
    threading.Thread(target=run_camera, args=(0, True), daemon=True).start()
    threading.Thread(target=run_camera, args=(1, False), daemon=True).start()

    print(f"[camerad] NV12 direct. {CAP_W}x{CAP_H}@{FPS} ds={DOWNSAMPLE} sync_tol={SYNC_TOL_MS}ms", flush=True)

    last_l = last_r = 0
    while True:
        snap = store.get_pair_best_effort()
        if not snap:
            print("[camerad] waiting for both cameras...", flush=True)
            time.sleep(1)
            continue

        l, r, lc, rc = snap
        dl = lc - last_l
        dr = rc - last_r
        last_l, last_r = lc, rc

        dms = pts_delta_ms(l, r)
        synced = abs(dms) <= SYNC_TOL_MS

        # Print proof-of-life and buffer sizes (so you know data is real)
        print(
            f"[camerad] +L{dl} +R{dr} pts_dt={dms:.2f}ms synced={synced} "
            f"L(Y={len(l.y)} UV={len(l.uv)} {l.w}x{l.h}) "
            f"R(Y={len(r.y)} UV={len(r.uv)} {r.w}x{r.h})",
            flush=True
        )
        time.sleep(2)


if __name__ == "__main__":
    main()
