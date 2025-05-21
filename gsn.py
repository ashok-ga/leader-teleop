#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gsn.py – Jetson-friendly GStreamer recorder with optional live viewer   2025-05-23 · v9
"""
import os, signal, threading, numpy as np, gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
Gst.init(None)


class GstreamerRecorder:
    def __init__(self, *, fps=30, live_view=False, verbose=True):
        self.fps, self.live_view, self.verbose = fps, live_view, verbose
        self._dur = Gst.SECOND // fps
        self._pts = 0
        self._pipe = self._src = None
        self._w = self._h = None
        self._shutdown_evt = threading.Event()
        self._is_running = False
        signal.signal(signal.SIGINT,  lambda *_: self.shutdown())
        signal.signal(signal.SIGTERM, lambda *_: self.shutdown())

    # ───────── legacy helper ─────────
    def start_recording(self, output_file="video.mp4"):
        if self._pipe:
            self.stop_recording()
        self._outfile = os.path.join(output_file)
        os.makedirs(os.path.dirname(self._outfile), exist_ok=True)
        if self.verbose:
            print(f"🔴  Will start at first frame → {self._outfile}")
        self._is_running = True

    # ───────── push RGBA frame ─────────
    def push_frame(self, rgba: np.ndarray):
        if self._shutdown_evt.is_set():
            return
        if self._pipe is None:
            if not self._is_running:
                self.start_recording(os.path.basename(self._outfile))
            self._build_pipeline(rgba.shape[1], rgba.shape[0])
        if rgba.shape[:2] != (self._h, self._w):
            raise ValueError(f"Frame size must stay at {self._w}×{self._h}.")
        buf = Gst.Buffer.new_allocate(None, rgba.nbytes, None)
        buf.fill(0, rgba.tobytes())
        buf.pts, buf.duration = self._pts, self._dur
        self._pts += self._dur
        self._src.emit("push-buffer", buf)

    # ───────── stop & flush ─────────
    def stop_recording(self):
        if not self._is_running:
            return
        if self.verbose:
            print("🛑  Sending EOS…")
        try:
            self._src.emit("end-of-stream")
        except GLib.Error:
            pass
        bus = self._pipe.get_bus()
        if bus:
            bus.timed_pop_filtered(1 * Gst.SECOND,
                                   Gst.MessageType.EOS | Gst.MessageType.ERROR)
        self._pipe.set_state(Gst.State.NULL)
        if self.verbose:
            print("🟢  Recording finished.")
        self._pipe = self._src = None
        self._pts = 0
        self._is_running = False

    def shutdown(self):
        if not self._shutdown_evt.is_set():
            self._shutdown_evt.set()
            self.stop_recording()

    # ───────── pipeline builder ─────────
    def _build_pipeline(self, w, h):
        self._w, self._h = w, h
        if self.verbose:
            print(f"ℹ️  Locked to {w}×{h}")
        caps = f"video/x-raw,format=RGBA,width={w},height={h},framerate={self.fps}/1"
        if self._has_nvenc():
            self._pipe = self._make_pipe(jetson=True,  caps=caps) \
                       or self._make_pipe(jetson=False, caps=caps)
        else:
            self._pipe = self._make_pipe(jetson=False, caps=caps)
        if self._pipe is None:
            raise RuntimeError("Failed to build GStreamer pipeline.")
        self._src = self._pipe.get_by_name("src")
        self._src.set_property("block", True)
        self._pipe.set_state(Gst.State.PLAYING)
        if self.verbose:
            print(f"🔴  Recording → {os.path.abspath(self._outfile)}")
        threading.Thread(target=self._bus_watch, daemon=True).start()

    def _bus_watch(self):
        bus = self._pipe.get_bus()
        while not self._shutdown_evt.is_set():
            msg = bus.timed_pop_filtered(100 * Gst.MSECOND,
                                         Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if msg and msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                print(f"❌  {err.message} — {dbg}")
                self.shutdown()
                break

    # ───────── helper: nvenc presence ─────────
    @staticmethod
    def _has_nvenc():
        return Gst.ElementFactory.find("nvv4l2h264enc") is not None

    # ───────── pipeline string factory ─────────
    def _make_pipe(self, *, jetson: bool, caps: str):
        record_branch = (
            'nvvidconv ! nvv4l2h264enc insert-sps-pps=true maxperf-enable=true '
            'bitrate=8000000 ! h264parse ! mp4mux faststart=true ! '
            f'filesink location="{self._outfile}" sync=false'
            if jetson else
            'videoconvert ! video/x-raw,format=I420 ! '
            'x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 bitrate=5000 ! '
            'h264parse config-interval=1 ! mp4mux faststart=true ! '
            f'filesink location="{self._outfile}" sync=false'
        )

        if self.live_view:
            sink = "autovideosink"
            pipeline_desc = (
                f'appsrc name=src is-live=true block=true format=time caps="{caps}" ! '
                'videoconvert ! video/x-raw,format=NV12 ! '
                'tee name=t ! queue ! ' +
                record_branch + ' '
                't. ! queue ! videoconvert ! ' + sink + ' sync=false'
            )
        else:
            pipeline_desc = (
                f'appsrc name=src is-live=true block=true format=time caps="{caps}" ! ' +
                record_branch
            )

        if self.verbose:
            print("Building pipeline:", pipeline_desc)
        try:
            pipe = Gst.parse_launch(pipeline_desc)
            pipe.set_state(Gst.State.READY)
            return pipe
        except GLib.Error as e:
            if self.verbose:
                print(f"⚠️  Pipeline build failed: {e.message}")
            return None
