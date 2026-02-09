#!/usr/bin/env python3
import struct, json, urllib.request
import numpy as np
import cv2

HOST = "127.0.0.1:8001"

def fetch(kind="rgb"):
    url = f"http://{HOST}/{kind}"
    blob = urllib.request.urlopen(url, timeout=2).read()
    hlen = struct.unpack("<I", blob[:4])[0]
    header = json.loads(blob[4:4+hlen].decode("utf-8"))
    payload = blob[4+hlen:]

    w = header["out"]["w"]
    h = header["out"]["h"]
    one_len = header["L"]["len"]

    L = np.frombuffer(payload[:one_len], dtype=np.uint8).reshape(h, w, 3)
    R = np.frombuffer(payload[one_len:one_len*2], dtype=np.uint8).reshape(h, w, 3)
    return header, L, R

if __name__ == "__main__":
    header, L, R = fetch("rgb")
    print("header:", header["out"], "L ts:", header["L"]["t_mono_ns"], "R ts:", header["R"]["t_mono_ns"])

    # save one frame from each for quick verification (OpenCV wants BGR)
    cv2.imwrite("/dev/shm/demo_L_rgb.png", L[:,:,::-1])
    cv2.imwrite("/dev/shm/demo_R_rgb.png", R[:,:,::-1])

    header, Lp, Rp = fetch("preview")
    cv2.imwrite("/dev/shm/demo_L_preview.png", Lp[:,:,::-1])
    cv2.imwrite("/dev/shm/demo_R_preview.png", Rp[:,:,::-1])

    print("wrote /dev/shm/demo_*_{rgb,preview}.png")
