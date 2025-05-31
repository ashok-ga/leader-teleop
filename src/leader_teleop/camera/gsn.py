import signal
import sys
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
            # f"valve name=gate drop=true ! "
            # -- valve keeps buffers back until you open it --
            f"{decode_chain}"  # nvjpegdec / nvv4l2decoder / (none)
            f"nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! "
            f"nvv4l2h264enc maxperf-enable=1 bitrate=6000000 ! "
            f"h264parse name=parser ! "
            f"queue name=queue ! "
            f"mp4mux name=mux ! "
            f"filesink name=sink location={self._output_file} sync=false"
        )

        if self._verbose:
            print("📹 Starting camera recording pipeline:")
            print("    ", pipeline_str)

        self._pipeline = Gst.parse_launch(pipeline_str)
        self._gate = self._pipeline.get_by_name("gate")
        self._parser = self._pipeline.get_by_name("parser")

        self._add_buffer_probe()
        self._pipeline.set_state(Gst.State.PLAYING)

        # Add a bus watch to handle messages including errors
        self._bus = self._pipeline.get_bus()
        # self._bus.add_signal_watch()
        # self._bus.connect("message", self._on_bus_message)

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
            # self._finalize_pipeline()  # error → clean up
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
            # self._finalize_pipeline()  # normal end

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

        return Gst.PadProbeReturn.OK  # continue processing
        # if self._verbose:
        #     print(f"⏱️ Frame Timestamp: {timestamp_s:.6f} sec")

    def rotate_to_new_file(self, next_filename: str):
        pipeline = self._pipeline
        bus = self._bus
        queue = pipeline.get_by_name("queue")
        old_mux = pipeline.get_by_name("mux")
        old_sink = pipeline.get_by_name("sink")

        q_src = queue.get_static_pad("src")
        q_sink = queue.get_static_pad("sink")

        # 1. Block upstream – keep the id so we remove it exactly once
        block_id = q_sink.add_probe(
            Gst.PadProbeType.BLOCK_DOWNSTREAM, lambda *_, **__: Gst.PadProbeReturn.OK
        )

        # 2. Push EOS correctly (down-stream event, so PUSH or use sink pad)
        q_src.push_event(Gst.Event.new_eos())

        # 3. Wait until old branch has drained
        msg = bus.timed_pop_filtered(
            Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS | Gst.MessageType.ERROR
        )
        if msg.type == Gst.MessageType.ERROR:
            print("❌ rotation failed:", msg.parse_error()[1])
            q_sink.remove_probe(block_id)
            return

        # 4. Remove old mux/sink **and release request pad**
        pad_to_release = old_mux.get_static_pad("video_0")
        old_mux.release_request_pad(pad_to_release)
        for e in (old_sink, old_mux):
            e.set_state(Gst.State.NULL)
            pipeline.remove(e)

        # 5. Create & link the fresh branch
        new_mux = Gst.ElementFactory.make("mp4mux", "mux")
        new_sink = Gst.ElementFactory.make("filesink", "sink")
        new_sink.props.location = next_filename
        new_sink.props.sync = False

        pipeline.add(new_mux)
        pipeline.add(new_sink)

        new_mux.link(new_sink)

        v_pad = new_mux.request_pad_simple("video_%u")  # new idiom
        assert q_src.link(v_pad) == Gst.PadLinkReturn.OK

        for e in (new_mux, new_sink):
            e.sync_state_with_parent()

        # 6. Un-block upstream **once**
        q_sink.remove_probe(block_id)

        self._pipeline.set_state(Gst.State.PLAYING)

        print("blocked:", q_src.is_blocked(), "blocking:", q_src.is_blocking())
        print(pipeline.get_state(0))
        self._f = new_sink  # keep handle up-to-date
        print("🔄 switched to", next_filename)

    # def rotate_to_new_file(self, next_filename: str):
    #     parser = self._pipeline.get_by_name("parser")
    #     queue = self._pipeline.get_by_name("queue")
    #     filesink = self._pipeline.get_by_name("sink")

    #     queue_src_pad = queue.get_static_pad("src")
    #     queue_sink_pad = queue.get_static_pad("sink")

    #     parser_src_pad = parser.get_static_pad("src")
    #     parser_sink_pad = parser.get_static_pad("sink")
    #     print("parser src pad:", parser_src_pad)
    #     # gate_src_pad = self._gate.get_static_pad("src")
    #     # print("gate src pad:", gate_src_pad)
    #     # gate_sink_pad = self._gate.get_static_pad("sink")
    #     # print("gate sink pad:", gate_sink_pad)
    #     fsink_pad = filesink.get_static_pad("sink")
    #     print("filesink pad:", fsink_pad)

    #     def _on_block_probe(pad, info):
    #         """Probe function to block the pad."""
    #         print("🔒 Pad blocked, waiting for unlinking…")

    #         queue_sink_pad.send_event(Gst.Event.new_eos())
    #         print("sent EOS")

    #         fsink_pad.add_probe(
    #             Gst.PadProbeType.EVENT_DOWNSTREAM | Gst.PadProbeType.BLOCK,
    #             _unlink_and_continue,
    #         )

    #         parser_src_pad.remove_probe(block_id)

    #         return Gst.PadProbeReturn.OK

    #     # STEP 1 – block pad BEFORE the chain to remove
    #     block_id = parser_src_pad.add_probe(Gst.PadProbeType.BLOCK, _on_block_probe)
    #     print("Block ID: ", block_id)

    #     def _unlink_and_continue(pad, info):
    #         """Unlink the old mux and continue with EOS."""
    #         # STEP 4 – take old mux & sink out
    #         print("🔄 Unlinking old mux and sink…")
    #         print(pad, info)
    #         print("pad name:", pad.get_name())
    #         print("info name:", info.get_event().type)

    #         old_mux = self._pipeline.get_by_name("mux")
    #         old_sink = self._pipeline.get_by_name("sink")
    #         print("old mux:", old_mux)
    #         print("old sink:", old_sink)

    #         print(queue.unlink(old_mux))

    #         for e in (old_sink, old_mux):
    #             e.set_state(Gst.State.NULL)
    #             self._pipeline.remove(e)

    #         # STEP 5 – create fresh mux & sink, wire them up
    #         new_mux = Gst.ElementFactory.make("mp4mux", "mux")
    #         new_sink = Gst.ElementFactory.make("filesink", "sink")
    #         new_sink.set_property("location", next_filename)
    #         new_sink.set_property("sync", False)

    #         print("new mux:", new_mux)
    #         print("new sink:", new_sink)

    #         self._pipeline.add(new_mux)
    #         self._pipeline.add(new_sink)
    #         new_mux.link(new_sink)
    #         for e in (new_mux, new_sink):
    #             e.sync_state_with_parent()
    #         print("new mux state:", new_mux.get_state(Gst.CLOCK_TIME_NONE))
    #         # print("new sink state:", new_sink.get_state(Gst.CLOCK_TIME_NONE))
    #         # relink gate → new_mux

    #         print("ehre1")
    #         print(queue.link(new_mux))

    #         print("ehre2")

    #         # gate_src_pad.link(new_mux.get_static_pad("sink"))

    #         # STEP 6 – unblock the upstream pad so data flows again
    #         print("🔄  switched to", new_sink.get_property("location"))

    #         return Gst.PadProbeReturn.OK

    def _toggle_valve(self):
        """Toggles the valve"""
        if self._gate:
            if self._gate.get_property("drop"):
                print("🔓 Opening valve…")
                self._gate.set_property("drop", False)
            else:
                print("🔓 closing valve…")
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
            print("ehjker")
            self._pipeline.send_event(Gst.Event.new_eos())
            print("📨 EOS sent; waiting for clean-up…")
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
        output_file="bot1.mp4",
        device="/dev/video0",
        width=1280,
        height=720,
        fps=30,
        caps="MJPG",  # Change to "YUY2" or "YUYV" if needed
        verbose=True,
    )

    # recorder2 = GstreamerCameraRecorder(
    #     sync_buffer=dsb.get_buffer("scene_camera_bottom"),
    #     output_file="top1.mp4",
    #     device="/dev/video6",
    #     width=1280,
    #     height=720,
    #     fps=30,
    #     caps="MJPG",  # Change to "YUY2" or "YUYV" if needed
    #     verbose=False,
    # )

    # recorder1._toggle_valve()  # Start recording
    # recorder2._toggle_valve()  # Start recording

    time.sleep(3)  # Record for 10 seconds

    # recorder1._toggle_valve()  # Stop recording
    recorder1.rotate_to_new_file("bot2.mp4")  # Rotate to a new file
    # recorder1._toggle_valve()  # Start recording

    # recorder2.rotate_to_new_file("top2.mp4")  # Rotate to a new file

    time.sleep(5)  # Record for 10 seconds

    recorder1.shutdown()  # Shutdown the recorder
    # recorder1.rotate_to_new_file("bot3.mp4")  # Rotate to a new file
    # time.sleep(5)
    # recorder2.shutdown()

    # print(dsb.get_buffer("scene_camera_bottom").data)  # Print collected timestamps
