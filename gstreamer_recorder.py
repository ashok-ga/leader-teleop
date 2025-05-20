import shutil
import gi, threading, time

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib


class GstreamerRecorder:
    SHM_PATH = "/tmp/gst-shm.sock"

    def __init__(self, width=2560, height=720, fps=30):
        Gst.init(None)

        # 1) Preview pipeline A: appsrc → tee → preview branch + shmsink branch
        self.caps = (
            f"video/x-raw,format=RGBA,width={width},height={height},framerate={fps}/1"
        )
        pipeline_a = (
            f"appsrc name=src is-live=true format=time block=true caps=\"{self.caps}\" ! tee name=t "
            # preview
            "t. ! queue leaky=downstream ! videoconvert ! autovideosink sync=false "
            # shared‐memory output
            f"t. ! queue ! videoconvert ! shmsink socket-path={self.SHM_PATH} wait-for-connection=false sync=true"
        )

        print(pipeline_a)

        self.pipeline_a = Gst.parse_launch(pipeline_a)
        self.appsrc = self.pipeline_a.get_by_name("src")

        # buffer‐timing
        self.duration = Gst.SECOND // fps
        self.timestamp = 0

        # start preview pipeline
        self.pipeline_a.set_state(Gst.State.PLAYING)

        # spin A’s bus in its own loop so errors show up
        self.loop = GLib.MainLoop()
        bus = self.pipeline_a.get_bus()
        bus.add_signal_watch()
        bus.connect(
            "message::error",
            lambda b, m: (print("A Error:", m.parse_error()), self.loop.quit()),
        )
        threading.Thread(target=self.loop.run, daemon=True).start()

        # record pipeline B is not created until needed
        self.pipeline_b = None

    def push_frame(self, frame_bytes):
        buf = Gst.Buffer.new_allocate(None, len(frame_bytes), None)
        buf.fill(0, frame_bytes)
        
        buf.pts = self.timestamp
        buf.duration = self.duration
        self.timestamp += self.duration
        # print(f"[DEBUG] Pushing frame with PTS: {buf.pts}, duration: {buf.duration}")

        ret = self.appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            print("⚠️ push-buffer failed:", ret)

    def start_recording(self, output_file="out.mp4"):
        self.timestamp = 0

        if self.pipeline_b:
            print("Already recording!")
            return

        # 2) Record pipeline B: shmsrc → encoder → filesink
        pipeline_b = (
            f"shmsrc socket-path={self.SHM_PATH} ! capsfilter caps=\"{self.caps}\" ! queue ! "
            "videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast ! "
            f"h264parse ! mp4mux ! filesink location={output_file} sync=false"
        )

        print(pipeline_b)

        self.pipeline_b = Gst.parse_launch(pipeline_b)
        bus = self.pipeline_b.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", lambda b, m: print("B Error:", m.parse_error()))

        self.pipeline_b.set_state(Gst.State.PLAYING)
        print("🔴 Recording started →", output_file)

    def stop_recording(self):
        if not self.pipeline_b:
            print("Not recording.")
            return

        # cleanly finish the file
        print("🛑 Sending EOS to pipeline_b...")

        self.pipeline_b.send_event(Gst.Event.new_eos())
        # wait for EOS message
        bus = self.pipeline_b.get_bus()

        print("🛑 Waiting for EOS message...")

        msg = bus.timed_pop_filtered(
            Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS | Gst.MessageType.ERROR
        )
        print("hjere", msg)
        if msg.type == Gst.MessageType.ERROR:
            print("B Error on EOS:", msg.parse_error())
        else:
            print("🟢 Recording finished cleanly")

        self.pipeline_b.set_state(Gst.State.NULL)
        self.pipeline_b = None

    def shutdown(self):
        # stop both pipelines
        if self.pipeline_b:
            self.stop_recording()

        self.appsrc.emit("end-of-stream")
        self.pipeline_a.set_state(Gst.State.NULL)
        self.loop.quit()


def main():
    import numpy as np

    rec = GstreamerRecorder()
    width, height = 2560, 720
    fps = 30
    num_frames = fps * 2  # 10 seconds

    rec.start_recording("temp.mp4")

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

            rec.push_frame(frame.tobytes())

            time.sleep(1 / fps)

    finally:
        # demo: toggle recording around here if you like
        rec.shutdown()


if __name__ == "__main__":
    main()
