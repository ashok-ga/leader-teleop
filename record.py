import gi
import threading
import sys

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

def record_pipeline(device, output):
    pipeline_str = (
        f'v4l2src device={device} ! '
        f'image/jpeg,framerate=30/1,width=1280,height=720 ! jpegdec ! '
        f'videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency ! '
        f'mp4mux ! filesink location={output}'
    )
    print("GStreamer pipeline:\n", pipeline_str)

    pipeline = Gst.parse_launch(pipeline_str)
    bus = pipeline.get_bus()
    pipeline.set_state(Gst.State.PLAYING)

    mainloop = GLib.MainLoop()
    eos_sent = threading.Event()

    def bus_call(bus, message, loop):
        mtype = message.type
        if mtype == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
            loop.quit()
        elif mtype == Gst.MessageType.EOS:
            print("End of stream")
            loop.quit()
        return True

    bus.add_signal_watch()
    bus.connect("message", bus_call, mainloop)

    # Thread to listen for user input
    def input_thread():
        print("Press 'q' and Enter to stop recording and save the file.")
        while True:
            inp = sys.stdin.readline().strip()
            if inp.lower() == 'q':
                print("Stopping pipeline...")
                eos_sent.set()
                pipeline.send_event(Gst.Event.new_eos())
                break

    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    try:
        mainloop.run()
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        pipeline.set_state(Gst.State.NULL)
        print("Pipeline stopped.")

def main():
    record_pipeline('/dev/video2', 'scene1.mp4')

if __name__ == '__main__':
    main()
