import os
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib

Gst.init(None)

PORT = os.environ.get("RTSP_PORT", "8554")
W = int(os.environ.get("CAM_W", "1920"))
H = int(os.environ.get("CAM_H", "1080"))
FPS = int(os.environ.get("CAM_FPS", "30"))
BITRATE = int(os.environ.get("BITRATE", "8000000"))  # bits/s

def make_factory(sensor_id):
    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_shared(True)

    # Hardware encode to H264, payload to RTP (RTSP will serve it)
    # Note: parentheses are required in the launch string
    launch = (
        f"( nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={W},height={H},format=NV12,framerate={FPS}/1 ! "
        f"nvv4l2h264enc bitrate={BITRATE} insert-sps-pps=true iframeinterval={FPS} ! "
        f"h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )
    factory.set_launch(launch)
    return factory

def main():
    server = GstRtspServer.RTSPServer()
    server.set_service(PORT)

    mounts = server.get_mount_points()
    mounts.add_factory("/cam0", make_factory(0))
    mounts.add_factory("/cam1", make_factory(1))

    server.attach(None)

    print(f"[streamd] RTSP server ready:", flush=True)
    print(f"[streamd]   rtsp://<JETSON_IP>:{PORT}/cam0", flush=True)
    print(f"[streamd]   rtsp://<JETSON_IP>:{PORT}/cam1", flush=True)

    loop = GLib.MainLoop()
    loop.run()

if __name__ == "__main__":
    main()
