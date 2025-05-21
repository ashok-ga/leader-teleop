import gi
import time
import cv2
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst


class GstreamerRecorder:
    def __init__(self, width=2560, height=720, fps=30, preview=True):
        Gst.init(None)
        self.width = width
        self.height = height
        self.fps = fps
        self.duration = Gst.SECOND // fps
        self.timestamp = 0

        self.pipeline = None
        self.appsrc = None

        self.preview = preview
        self.window_name = "Preview"

        # OpenCV preview window
        if self.preview:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.width , self.height)

    def start_recording(self, output_file="output.mp4"):
        self.timestamp = 0
        if self.pipeline:
            print("⚠️ Already recording.")
            return

        caps = (
            f"video/x-raw,format=RGBA,width={self.width},"
            f"height={self.height},framerate={self.fps}/1"
        )

        # 1) Convert RGBA→I420, 2) encode, 3) inline SPS/PPS, 4) fast-start MP4
        pipeline_desc = (
            f'appsrc name=appsrc is-live=true format=time block=true caps="{caps}" ! '
            # ensure we’re in a format x264 wants
            "videoconvert ! video/x-raw,format=I420 ! "
            # encode
            "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 bitrate=5000 ! "
            # inline SPS/PPS before each keyframe
            "h264parse config-interval=1 ! "
            # put metadata at the front of the file so players see it immediately
            "mp4mux faststart=true ! "
            "filesink location={} sync=false"
        ).format(output_file)

        self.pipeline = Gst.parse_launch(pipeline_desc)
        self.appsrc = self.pipeline.get_by_name("appsrc")

        self.pipeline.set_state(Gst.State.PLAYING)
        print(f"🔴 Recording started to {output_file}")

    def push_frame(self, frame_rgba: np.ndarray):
        if self.appsrc is None:
            return

        # Push into GStreamer
        data = frame_rgba.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.pts = self.timestamp
        buf.duration = self.duration
        self.timestamp += self.duration

        ret = self.appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            print("⚠️ push-buffer failed:", ret)
        # Show preview via OpenCV
        if self.preview:
            # Convert RGBA to BGR for display
            print("1")
            bgr = cv2.cvtColor(frame_rgba, cv2.COLOR_RGBA2BGR)
            
            print("2")
            cv2.imshow(self.window_name, bgr)
            
            print("3")
            cv2.waitKey(1)


    def stop_recording(self):
        if not self.pipeline:
            print("Not recording.")
            return

        print("🛑 Sending EOS...")
        self.pipeline.send_event(Gst.Event.new_eos())
        bus = self.pipeline.get_bus()
        msg = bus.timed_pop_filtered(
            Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS | Gst.MessageType.ERROR
        )

        if msg.type == Gst.MessageType.ERROR:
            print("❌ Recording error:", msg.parse_error())
        else:
            print("🟢 Recording finished cleanly.")

        self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None
        self.appsrc = None

    def shutdown(self):
        if self.pipeline:
            self.stop_recording()

        # Destroy preview window
        if self.preview:
            cv2.destroyWindow(self.window_name)


def main():
    import numpy as np

    rec = GstreamerRecorder()
    width, height = 2560, 720
    fps = 30
    num_frames = fps * 2  # 10 seconds

    rec.start_recording("output.mp4")

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

            rec.push_frame(frame)

            time.sleep(1 / fps)

    finally:
        # demo: toggle recording around here if you like
        rec.shutdown()


if __name__ == "__main__":
    main()
