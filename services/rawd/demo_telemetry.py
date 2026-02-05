#!/usr/bin/env python3
import time
import urllib.request
import urllib.error

JETSON = "http://127.0.0.1:8001"  # on Jetson, talk to localhost
TIMEOUT_S = 2.0

def http_get(path: str) -> bytes:
    with urllib.request.urlopen(JETSON + path, timeout=TIMEOUT_S) as r:
        return r.read()

def http_post(path: str, data: bytes, content_type="application/octet-stream") -> bytes:
    req = urllib.request.Request(
        JETSON + path,
        data=data,
        method="POST",
        headers={"Content-Type": content_type, "Content-Length": str(len(data))},
    )
    with urllib.request.urlopen(req, timeout=TIMEOUT_S) as r:
        return r.read()

def main():
    print("Demo telemetry: Teensy->Jetson and UI->Jetson buffers")

    # 1) Send "teensy telemetry" bytes into rawd
    teensy_payload = b"TEENSY: steering=0.12 throttle=0.34 t_ms=123456"
    try:
        resp = http_post("/telemetry/teensy_in", teensy_payload)
        print("POST /telemetry/teensy_in ->", resp[:200])
    except urllib.error.URLError as e:
        print("ERROR posting teensy_in:", e)

    # 2) Read status (shows if telemetry buffers are present)
    try:
        status = http_get("/status")
        print("GET /status ->", status[:300])
    except urllib.error.URLError as e:
        print("ERROR reading status:", e)

    # 3) Send "UI->Jetson command" bytes into rawd
    ui_payload = b"UI: set_speed=0.50 set_turn=-0.20"
    try:
        resp = http_post("/telemetry/ui_in", ui_payload)
        print("POST /telemetry/ui_in ->", resp[:200])
    except urllib.error.URLError as e:
        print("ERROR posting ui_in:", e)

    # 4) Read back what rawd currently has (if your rawd exposes GET endpoints)
    # If your rawd doesn’t have these GET endpoints yet, you can skip this section.
    for p in ("/telemetry/ui_in", "/telemetry/teensy_in"):
        try:
            b = http_get(p)
            print(f"GET {p} -> {b[:200]}")
        except Exception as e:
            print(f"(Info) GET {p} not available or failed:", e)

    print("Done.")

if __name__ == "__main__":
    main()
