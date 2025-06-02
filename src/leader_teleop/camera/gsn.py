#!/usr/bin/env python3
import signal, threading, time, datetime
from gi.repository import Gst, GLib
from leader_teleop.buffer.data_sync_buffer import Buffer, DataSyncBuffer

from gi.repository import Gst, GLib
from gi.repository import Gst, GLib, GstVideo
import inspect


Gst.init(None)


class GstreamerCameraRecorder:
    def __init__(
        self,
        sync_buffer: Buffer | None = None,
        output_pattern: str = "camera_%05d.mp4",  # pattern for splitmuxsink
        device: str = "/dev/video0",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        caps: str = "MJPG",
        verbose: bool = True,
    ):
        self.sync_buffer = sync_buffer
        self._verbose = verbose

        # ───────────────── camera source caps ─────────────────
        caps = caps.upper()
        if caps == "MJPG":
            src_caps = f"image/jpeg,width={width},height={height},framerate={fps}/1"
            decode_chain = "jpegparse ! nvv4l2decoder ! "
        elif caps in {"YUY2", "YUYV"}:
            src_caps, decode_chain = (
                f"video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1",
                "",
            )
        else:
            raise ValueError(f"Unsupported caps format: {caps}")

        # ───────────────── pipeline with splitmuxsink ─────────────────
        pipe_str = (
            f"v4l2src device={device} ! {src_caps}  ! "
            "valve name=gate drop=true ! "
            f"{decode_chain}"
            # ───────── NVMM → system-memory ─────────
            # "nvvidconv nvbuf-memory-type=2 ! video/x-raw,format=I420 ! "
            # ───────── draw clock overlay ─────────
            # "timeoverlay "
            # "valignment=top halignment=left shaded-background=true "
            # 'font-desc="Sans, 24" ! '
            # ───────── system-memory → NVMM ─────────
            "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
            # ───────── HW H.264 encode + rotation ─────────
            "nvv4l2h264enc maxperf-enable=1 control-rate=1 bitrate=8000000 "
            "iframeinterval=1 insert-sps-pps=1 ! "
            "h264parse name=parser ! "
            "splitmuxsink name=smux muxer=mp4mux "
            "send-keyframe-requests=true "
            "max-size-time=0 max-size-bytes=0 "
            f"location={output_pattern} async-finalize=false"
        )

        # pipe_str = (
        #     f"v4l2src device={device} do-timestamp=true ! {src_caps}  ! "
        #     "valve name=gate drop=true ! "
        #     "videoconvert ! "
        #     # ───────── NVMM → system-memory ─────────
        #     # "nvvidconv nvbuf-memory-type=2 ! video/x-raw,format=I420 ! "
        #     # ───────── draw clock overlay ─────────
        #     # "timeoverlay "
        #     # "valignment=top halignment=left shaded-background=true "
        #     # 'font-desc="Sans, 24" ! '
        #     # ───────── system-memory → NVMM ─────────
        #     # "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
        #     # ───────── HW H.264 encode + rotation ─────────
        #     "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=1 ! "
        #     "h264parse name=parser ! "
        #     "splitmuxsink name=smux muxer=mp4mux "
        #     "send-keyframe-requests=true "
        #     "max-size-time=0 max-size-bytes=0 "
        #     f"location={output_pattern} async-finalize=false"
        # )

        if verbose:
            print("📹 GStreamer pipeline:")
            print("   ", pipe_str)

        self._pipeline = Gst.parse_launch(pipe_str)
        self._gate = self._pipeline.get_by_name("gate")
        self._smux = self._pipeline.get_by_name("smux")

        # optional: give each new file a timestamped name instead of %05d
        # self._smux.connect("format-location", self._on_format_location)

        self._add_buffer_probe()  # timestamps into DataSyncBuffer
        self._bus = self._pipeline.get_bus()
        self._bus.add_signal_watch()
        self._bus.connect("message", self._on_bus)

        self._pipeline.set_state(Gst.State.PLAYING)

        self._mainloop = GLib.MainLoop()
        self._loop_thread = threading.Thread(target=self._mainloop.run, daemon=True)
        self._loop_thread.start()

        signal.signal(signal.SIGINT, lambda *_: self.shutdown())
        signal.signal(signal.SIGTERM, lambda *_: self.shutdown())

        self.recording = False

    # ───────────────────────── helpers ─────────────────────────
    def _on_format_location(self, mux, idx):
        # idx is the split index starting at 0 – build your own file name here
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"{ts}_{idx:03d}.mp4"

    def _add_buffer_probe(self):
        parser = self._pipeline.get_by_name("parser")
        parser_src = parser.get_static_pad("sink")

        def _probe_cb(pad, info):
            buf = info.get_buffer()
            self.sync_buffer.add(buf.pts / Gst.SECOND)
            return Gst.PadProbeReturn.OK

        parser_src.add_probe(Gst.PadProbeType.BUFFER, _probe_cb)

    def _on_bus(self, bus, msg):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print("❌", err, dbg or "")
            self._finalise()
        elif t == Gst.MessageType.EOS:
            if self._verbose:
                print("✅ EOS received")
            self._finalise()

    # ───────────────────────── teardown ─────────────────────────
    def shutdown(self):
        if self._verbose:
            print("🧹 Shutting down …")
        if not self._pipeline:
            return
        self._pipeline.send_event(Gst.Event.new_eos())  # async; _on_bus handles NULL

    def _finalise(self):
        if getattr(self, "_done", False):
            return
        self._done = True
        self._pipeline.set_state(Gst.State.NULL)
        self._bus.remove_signal_watch()
        self._pipeline = None
        if self._mainloop.is_running():
            self._mainloop.quit()

    def _force_key_unit(self):
        pad = self._gate.get_static_pad("src")  # valve must be OPEN
        if not pad:
            return
        rt = self._pipeline.get_clock().get_time() - self._pipeline.get_base_time()

        # JetPack has the 3-arg legacy helper:
        try:
            from gi.repository import GstVideo

            ev = GstVideo.video_event_new_upstream_force_key_unit(rt, True, 0)
        except Exception:
            st = Gst.Structure.new_empty("GstForceKeyUnit")
            st.set_value("all-headers", True)
            st.set_value("count", 0)
            v = GLib.Value("guint64")
            v.set_uint64(rt)
            st.set_value("timestamp", v)
            ev = Gst.Event.new_custom(Gst.EventType.CUSTOM_UPSTREAM, st)

        pad.send_event(ev)

    def start_recording(self):
        """Start recording by opening the valve."""
        self._gate.set_property("drop", False)
        self._is_recording = True
        print("🔴 Recording started")

    def is_recording(self):
        return self.recording

    def stop_recording(self, final=False):
        """
        If final=True is not given for the last recording, it will be unplayable
        because the splitmuxsink will not write the moov atom.
        """
        self._force_key_unit()  # next frame is an IDR
        self._smux.emit("split-after")  # close current file cleanly

        self.recording = False

        if final:
            # keep the valve OPEN so EOS can travel downstream
            self._pipeline.send_event(Gst.Event.new_eos())  # whole pipe
            return

        # not final: close valve after ≥1 frame
        GLib.timeout_add(
            40, lambda *_: (self._gate.set_property("drop", True) or False)
        )
        print("🟡 Recording stopped")


# ─────────────────────────── demo ───────────────────────────
if __name__ == "__main__":
    dsb = DataSyncBuffer(
        ["scene_camera_bottom", "scene_camera_top", "wrist_camera_right"]
    )

    # import multiprocessing as mp

    # cfg = {
    #     "output_pattern": "bot_%d.mp4",
    #     "device": "/dev/video0",
    #     "width": 1280,
    #     "height": 720,
    #     "fps": 30,
    #     "caps": "MJPG",
    # }
    # mp.Process(target=worker, args=(cfg,)).start()

    # cfg = {
    #     "output_pattern": "top_%d.mp4",
    #     "device": "/dev/video4",
    #     "width": 1280,
    #     "height": 720,
    #     "fps": 30,
    #     "caps": "MJPG",
    # }
    # mp.Process(target=worker, args=(cfg,)).start()

    # cfg = {
    #     "output_pattern": "right_%d.mp4",
    #     "device": "/dev/video8",
    #     "width": 2560,
    #     "height": 720,
    #     "fps": 30,
    #     "caps": "YUYV",
    # }
    # mp.Process(target=worker, args=(cfg,)).start()

    # rec1 = GstreamerCameraRecorder(
    #     sync_buffer=dsb.get_buffer("scene_camera_bottom"),
    #     output_pattern="bot_%d.mp4",
    #     device="/dev/video0",
    #     width=1280,
    #     height=720,
    #     fps=30,
    #     caps="MJPG",
    # )
    # rec2 = GstreamerCameraRecorder(
    #     sync_buffer=dsb.get_buffer("scene_camera_top"),
    #     output_pattern="top_%d.mp4",
    #     device="/dev/video4",
    #     width=1280,
    #     height=720,
    #     fps=30,
    #     caps="MJPG",
    #     verbose=False,
    # )
    rec3 = GstreamerCameraRecorder(
        sync_buffer=dsb.get_buffer("wrist_camera_right"),
        output_pattern="right_%d.mp4",
        device="/dev/video8",
        width=2560,
        height=720,
        fps=30,
        caps="YUYV",
        verbose=False,
    )

    time.sleep(3)  # camera warm-up

    # rec1.start_recording()  # open the valve, start recording
    # rec2.start_recording()
    for i in range(7):
        rec3.start_recording()
        time.sleep(2.3)
        rec3.stop_recording()
        time.sleep(1.2)

    rec3.shutdown()

    # rec1.stop_recording()  # close the valve, stop recording
    # rec2.stop_recording()

    # time.sleep(1)  # wait for the splitmuxsink to finish writing

    # rec1.start_recording()  # open the valve again, start recording
    # rec2.start_recording()

    # print("recoding 2 now")

    # time.sleep(5)

    # rec1.stop_recording()  # close the valve, stop recording
    # rec2.stop_recording()

    # time.sleep(1)

    # rec1.start_recording()
    # rec2.start_recording()

    # time.sleep(3.1)

    # rec1.stop_recording()  # close the valve, stop recording and finalize
    # rec2.stop_recording()
    # rec3.stop_recording()

    # rec1.shutdown()
    # rec2.shutdown()
    # rec3.shutdown()

    # print(dsb.get_buffer("scene_camera_bottom").data)
