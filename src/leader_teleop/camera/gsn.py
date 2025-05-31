import signal
import threading
import time
from gi.repository import Gst, GLib

from leader_teleop.buffer.data_sync_buffer import Buffer, DataSyncBuffer

Gst.init(None)


class GstreamerCameraRecorder:
    def __init__(
        self,
        sync_buffer: Buffer = None,
        output_file: str = "output.mp4",
        device: str = "/dev/video0",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        caps: str = "MJPG",
        verbose: bool = True,
    ):
        self.sync_buffer = sync_buffer
        self._device = device
        self._output_file = output_file
        self._width = width
        self._height = height
        self._fps = fps
        self._caps = caps.upper()  # normalise
        self._verbose = verbose

        # ------------------------------------------------------------
        # Build pipeline string based on the camera's pixel format
        # ------------------------------------------------------------
        if self._caps == "MJPG":  # Motion-JPEG
            src_caps = (
                f"image/jpeg,width={self._width},height={self._height},"
                f"framerate={self._fps}/1"
            )
            # NOTE: For some reason using `jpegdec` directly here instead of `nvjpegdec`
            # results in a better CPU usage (60% comapred to 120%).
            # With 'nvjpegdec' the CPU usage is higher, and the frame rate drops.
            # Turns out jpegparse + nvv4l2decoder is the best option (25% CPU usage).
            decode_chain = "jpegparse ! nvv4l2decoder ! "
            # decode_chain = "jpegdec !"
        elif self._caps in {"YUY2", "YUYV"}:  # Uncompressed YUY2/YUYV
            src_caps = (
                f"video/x-raw,format=YUY2,width={self._width},height={self._height},"
                f"framerate={self._fps}/1"
            )
            decode_chain = ""  # already raw, no decoder needed
        else:
            raise ValueError(f"Unsupported caps format: {self._caps}")

        pipeline_str = (
            f"v4l2src device={self._device} ! {src_caps} ! "
            # -- valve keeps buffers back until you open it --
            f"{decode_chain}"  # nvjpegdec / nvv4l2decoder / (none)
            f"nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! "
            f"nvv4l2h264enc maxperf-enable=1 bitrate=6000000 ! "
            f"h264parse name=parser ! "
            f"valve name=gate drop=true ! "
            f"mp4mux ! "
            f"filesink location={self._output_file} sync=false"
        )

        if self._verbose:
            print("📹 Starting camera recording pipeline:")
            print("    ", pipeline_str)

        self._pipeline = Gst.parse_launch(pipeline_str)
        self._gate = self._pipeline.get_by_name("gate")

        self._add_buffer_probe()
        self._pipeline.set_state(Gst.State.PLAYING)

        # Add a bus watch to handle messages including errors
        self._bus = self._pipeline.get_bus()
        self._bus.add_signal_watch()
        self._bus.connect("message", self._on_bus_message)

        self._mainloop = GLib.MainLoop()
        self._loop_thread = threading.Thread(target=self._mainloop.run, daemon=True)
        self._loop_thread.start()

        signal.signal(signal.SIGINT, lambda *_: self.shutdown())
        signal.signal(signal.SIGTERM, lambda *_: self.shutdown())

        # print(self._pipeline.get_state(Gst.CLOCK_TIME_NONE))

    def _on_bus_message(self, bus, message):
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"❌ GStreamer Pipeline Error: {err}")
            if debug:
                print(f"🔍 Debug Info: {debug}")
            self._finalize_pipeline()  # error → clean up
        elif message.type == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            print(f"⚠️ GStreamer Warning: {warn}")
            if debug:
                print(f"🔍 Debug Info: {debug}")
        elif message.type == Gst.MessageType.INFO:
            info, debug = message.parse_info()
            print(f"ℹ️ GStreamer Info: {info}")
            if debug:
                print(f"🔍 Debug Info: {debug}")
        elif message.type == Gst.MessageType.EOS:
            if self._verbose:
                print("✅ EOS received (async)")
            self._finalize_pipeline()  # normal end

    def _add_buffer_probe(self):
        # Get the element by name and attach probe on its src pad
        parser = self._pipeline.get_by_name("parser")
        if not parser:
            print("❌ Could not find parser element for attaching probe.")
            return

        srcpad = parser.get_static_pad("src")
        if not srcpad:
            print("❌ Could not get src pad from parser.")
            return

        srcpad.add_probe(Gst.PadProbeType.BUFFER, self._on_buffer_probe)

    def _on_buffer_probe(self, pad, info):
        buffer = info.get_buffer()
        if not buffer:
            return Gst.PadProbeReturn.OK

        timestamp_ns = buffer.pts
        timestamp_s = timestamp_ns / Gst.SECOND

        self.sync_buffer.add(timestamp_s)
        # if self._verbose:
        #     print(f"⏱️ Frame Timestamp: {timestamp_s:.6f} sec")

        return Gst.PadProbeReturn.OK

    def arm(self):
        """Toggles the valve"""
        if self._gate:
            if self._gate.get_property("drop"):
                print("🔓 Opening valve…")
                self._gate.set_property("drop", False)
            else:
                print("🔓 closing valve…")
                if self._verbose:
                    print("📽️ Sending EOS event to pipeline...")
                self._gate.set_property("drop", True)

    # centralised clean-up
    def _finalize_pipeline(self):
        if getattr(self, "_already_finalised", False):
            return  # run only once
        self._already_finalised = True

        # # stop main-loop if you started one
        # if hasattr(self, "_mainloop") and self._mainloop.is_running():
        #     self._mainloop.quit()

        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)
            if self._bus:
                self._bus.remove_signal_watch()
            self._pipeline = None

    # ------------------------------------------------------------------
    # non-blocking shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        if self._verbose:
            print("🧹 Requesting pipeline shutdown…")

        if not self._pipeline:
            return

        # Send EOS only if the pipeline is running
        _, state, _ = self._pipeline.get_state(timeout=0)
        if state in (Gst.State.PLAYING, Gst.State.PAUSED):
            self._pipeline.send_event(Gst.Event.new_eos())
            if self._verbose:
                print("📨 EOS sent; returning immediately (async clean-up).")
        else:
            # if already in READY/NULL just finalise right away
            self._finalize_pipeline()


def create_pipeline(
    device: str,
    output_file: str,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    caps: str = "MJPG",
):
    """Create a GStreamer pipeline for camera recording."""
    recorder = GstreamerCameraRecorder(
        sync_buffer=None,  # No sync buffer in this standalone function
        output_file=output_file,
        device=device,
        width=width,
        height=height,
        fps=fps,
        caps=caps,
        verbose=True,
    )
    return recorder


if __name__ == "__main__":
    dsb = DataSyncBuffer(["scene_camera_bottom", "scene_camera_top"])
    recorder1 = GstreamerCameraRecorder(
        sync_buffer=dsb.get_buffer("scene_camera_bottom"),
        output_file="camera_recorded1.mp4",
        device="/dev/video0",
        width=1280,
        height=720,
        fps=30,
        caps="MJPG",  # Change to "YUY2" or "YUYV" if needed
        verbose=True,
    )

    recorder2 = GstreamerCameraRecorder(
        sync_buffer=dsb.get_buffer("scene_camera_bottom"),
        output_file="camera_recorded2.mp4",
        device="/dev/video6",
        width=1280,
        height=720,
        fps=30,
        caps="MJPG",  # Change to "YUY2" or "YUYV" if needed
        verbose=False,
    )

    time.sleep(2)  # Allow time for pipeline to initialize

    recorder1.arm()  # Start recording
    recorder2.arm()  # Start recording

    time.sleep(5)
    recorder1.shutdown()
    recorder2.shutdown()

    print(dsb.get_buffer("scene_camera_bottom").data)  # Print collected timestamps
