import threading
import time
import gi
import platform

import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib


class GstreamerRecorder:
    def _start_main_loop_with_bus(self):
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)
        self.loop.run()

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"🚨 GStreamer Error: {err}, Debug: {debug}")
            self.loop.quit()
        elif t == Gst.MessageType.EOS:
            print("✅ Received EOS")
            self.loop.quit()

    def __init__(self, output_file="output.mp4"):
        Gst.init(None)
        self.timestamp = 0
        self.duration = Gst.SECOND // 30  # 30 FPS

        pipeline_str = (
            "appsrc name=source is-live=true block=true format=time "
            "caps=video/x-raw,format=RGBA,width=2560,height=720,framerate=30/1 ! tee name=t "
            # Preview branch
            "t. ! queue leaky=downstream ! videoconvert ! autovideosink sync=false "
            # Recording branch
            "t. ! queue ! videoconvert ! "
            "x264enc ! h264parse ! mp4mux ! "
            "filesink location={} sync=false"
        ).format(output_file)

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("source")

        if not self.appsrc:
            raise RuntimeError("Failed to create GStreamer appsrc.")

        self.appsrc.set_property("format", Gst.Format.TIME)
        self.appsrc.set_property("block", True)

        self.pipeline.set_state(Gst.State.PLAYING)
        print(f"🎥 Recording started: {output_file}")

        self.loop = GLib.MainLoop()
        t = threading.Thread(target=self._start_main_loop_with_bus, daemon=True)
        t.start()

    def push_frame(self, frame):
        print("Pushing frame...")
        try:
            buf = Gst.Buffer.new_allocate(None, len(frame), None)
            buf.fill(0, frame)
            buf.pts = self.timestamp
            buf.duration = self.duration
            self.timestamp += self.duration

            ret = self.appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                print(f"⚠️ Push-buffer error: {ret}")

            print("Frame pushed successfully.")
        except Exception as e:
            print(f"⚠️ push_frame exception: {e}")

    def close(self):
        print("Closing GStreamer pipeline...")
        try:
            # stop the streams
            self.appsrc.emit("end-of-stream")
            # tell the mainloop to quit (which also stops the video window)
            self.loop.quit()
            # now wait for EOS and tear down
            bus = self.pipeline.get_bus()
            msg = bus.timed_pop_filtered(
                Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS | Gst.MessageType.ERROR
            )
            # … handle msg as before …
            self.pipeline.set_state(Gst.State.NULL)
            print("Recording stopped successfully.")
        except Exception as e:
            print(f"⚠️ close exception: {e}")


def main():
    # write to "test_output.mp4" and preview via autovideosink
    recorder = GstreamerRecorder(output_file="test_output.mp4")

    width, height = 2560, 720
    fps = 30
    num_frames = fps * 2  # 10 seconds

    try:
        for _ in range(num_frames):
            print("Recording frame...")
            # generate a simple moving gradient frame
            x = np.linspace(0, 255, width, dtype=np.uint8)
            row = np.stack(
                [x, x[::-1], np.zeros_like(x), 255 * np.ones_like(x)], axis=1
            )
            frame = np.repeat(row[np.newaxis, :, :], height, axis=0)
            print(f"Frame shape: {frame.shape}, size: {len(frame.tobytes())}")

            recorder.push_frame(frame.tobytes())

            time.sleep(1 / fps)
    finally:
        recorder.close()


if __name__ == "__main__":
    main()
