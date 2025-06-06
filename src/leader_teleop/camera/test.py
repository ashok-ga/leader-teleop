#!/usr/bin/env python3
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib, GObject

import time

Gst.init(None)


class LivePipeline:
    def __init__(self):
        self.pipeline = Gst.Pipeline.new("live-pipeline")

        # Source
        self.src = Gst.ElementFactory.make("videotestsrc", "src")
        self.src.set_property("is-live", True)
        self.tee = Gst.ElementFactory.make("tee", "tee")

        self.pipeline.add(self.src)
        self.pipeline.add(self.tee)
        self.src.link(self.tee)

        # Preview branch
        self.preview_queue = Gst.ElementFactory.make("queue", "preview_queue")
        self.preview_sink = Gst.ElementFactory.make("fakesink", "preview_sink")

        for el in [self.preview_queue, self.preview_sink]:
            self.pipeline.add(el)

        tee_pad_preview = self.tee.get_request_pad("src_%u")
        tee_pad_preview.link(self.preview_queue.get_static_pad("sink"))
        self.preview_queue.link(self.preview_sink)

        # Recording valve
        self.tee_pad_record = self.tee.get_request_pad("src_%u")
        self.valve = Gst.ElementFactory.make("valve", "valve")
        self.pipeline.add(self.valve)
        self.tee_pad_record.link(self.valve.get_static_pad("sink"))

        # Initial file
        self.build_recording_branch("output1.mp4")

    def build_recording_branch(self, file_path):
        self.queue = Gst.ElementFactory.make("queue", None)
        self.encoder = Gst.ElementFactory.make("x264enc", None)
        self.mux = Gst.ElementFactory.make("mp4mux", None)
        self.filesink = Gst.ElementFactory.make("filesink", None)
        self.filesink.set_property("location", file_path)

        for el in [self.queue, self.encoder, self.mux, self.filesink]:
            self.pipeline.add(el)

        self.valve.link(self.queue)
        self.queue.link(self.encoder)
        self.encoder.link(self.mux)
        self.mux.link(self.filesink)

        for el in [self.queue, self.encoder, self.mux, self.filesink]:
            el.sync_state_with_parent()

        self.valve.set_property("drop", False)

    def switch_file(self, new_path):
        self.valve.set_property("drop", True)
        print("Sending EOS to file 1 branch...")
        self.valve.get_static_pad("src").send_event(Gst.Event.new_eos())
        time.sleep(1.0)

        print("Tearing down old branch")
        self.valve.unlink(self.queue)
        self.queue.unlink(self.encoder)
        self.encoder.unlink(self.mux)
        self.mux.unlink(self.filesink)

        for el in [self.queue, self.encoder, self.mux, self.filesink]:
            self.pipeline.remove(el)

        self.build_recording_branch(new_path)

    def finalize_last_file(self):
        print("Finalizing second file with EOS...")
        self.valve.set_property("drop", True)
        self.valve.get_static_pad("src").send_event(Gst.Event.new_eos())

    def wait_for_eos_flush(self):
        bus = self.pipeline.get_bus()
        while True:
            msg = bus.timed_pop_filtered(
                5 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR
            )
            if msg:
                if msg.type == Gst.MessageType.EOS:
                    print("EOS received, file finalized.")
                    break
                elif msg.type == Gst.MessageType.ERROR:
                    err, debug = msg.parse_error()
                    print(f"ERROR: {err}, debug: {debug}")
                    break
            else:
                print("EOS timeout.")
                break

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)


def main():
    GObject.threads_init()
    pipeline = LivePipeline()
    pipeline.start()

    print("Recording to output1.mp4")
    time.sleep(5)

    print("Switching to output2.mp4")
    pipeline.switch_file("output2.mp4")
    print("Recording to output2.mp4")
    time.sleep(5)

    print("Finalizing output2.mp4...")
    pipeline.finalize_last_file()
    pipeline.wait_for_eos_flush()

    print("Shutting down")
    pipeline.stop()


if __name__ == "__main__":
    main()
