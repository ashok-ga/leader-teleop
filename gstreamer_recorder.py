import gi
import platform
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class GstreamerRecorder:
    def __init__(self, output_file="output.mp4"):
        Gst.init(None)
        self.timestamp = 0
        self.duration = Gst.SECOND // 30  # 30 FPS

        pipeline_str = (
            "appsrc name=source is-live=true block=true format=time "
            "caps=video/x-raw,format=RGBA,width=2560,height=720,framerate=30/1 ! "
            "nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! "
            "nvv4l2h264enc bitrate=5000000 ! h264parse ! mp4mux ! "
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

    def push_frame(self, frame):
        try:
            buf = Gst.Buffer.new_allocate(None, len(frame), None)
            buf.fill(0, frame)
            buf.pts = self.timestamp
            buf.duration = self.duration
            self.timestamp += self.duration

            ret = self.appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                print(f"⚠️ Push-buffer error: {ret}")
        except Exception as e:
            print(f"⚠️ push_frame exception: {e}")

    def close(self):
        try:
            self.appsrc.emit("end-of-stream")

            bus = self.pipeline.get_bus()
            while True:
                msg = bus.timed_pop_filtered(
                    Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS | Gst.MessageType.ERROR
                )
                if msg:
                    if msg.type == Gst.MessageType.ERROR:
                        err, debug = msg.parse_error()
                        print(f"⚠️ GStreamer Error: {err}, Debug: {debug}")
                        break
                    elif msg.type == Gst.MessageType.EOS:
                        print("EOS received, finalizing video.")
                        break

            self.pipeline.set_state(Gst.State.NULL)
            print("Recording stopped successfully.")
        except Exception as e:
            print(f"⚠️ close exception: {e}")