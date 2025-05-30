import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import threading

Gst.init(None)

def record_with_preview(device="/dev/video2", outfile="scene1.mp4", duration=None, live_view=True, jetson=False):
    # Hardware or software encoding based on Jetson flag
    if jetson:
        record_branch = (
            'nvvidconv ! nvv4l2h264enc insert-sps-pps=true maxperf-enable=true '
            'bitrate=8000000 ! h264parse ! mp4mux faststart=true ! '
            f'filesink location={outfile} sync=false'
        )
    else:
        record_branch = (
            'videoconvert ! video/x-raw,format=I420 ! '
            'x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 bitrate=5000 ! '
            'h264parse config-interval=1 ! mp4mux faststart=true ! '
            f'filesink location={outfile} sync=false'
        )
    # Live preview with tee
    if live_view:
        pipeline_str = (
            f'v4l2src device={device} ! image/jpeg,framerate=30/1,width=1280,height=720 ! '
            'jpegdec ! tee name=t ! queue ! '
            f'{record_branch} '
            't. ! queue ! videoconvert ! autovideosink sync=false'
        ) 
    else:
        pipeline_str = (
            f'v4l2src device={device} ! image/jpeg,framerate=30/1,width=1280,height=720 ! '
            'jpegdec ! ' + record_branch
        )
    print("Pipeline:\n", pipeline_str)
    pipeline = Gst.parse_launch(pipeline_str)
    bus = pipeline.get_bus()
    pipeline.set_state(Gst.State.PLAYING)

    mainloop = GLib.MainLoop()

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

    def wait_for_q():
        print("Press 'q' then Enter to stop recording and save the file.")
        while True:
            inp = input().strip().lower()
            if inp == "q":
                print("Stopping...")
                pipeline.send_event(Gst.Event.new_eos())
                break

    # If you want a time limit, call pipeline.send_event(Gst.Event.new_eos()) after duration
    if duration:
        def stop_after():
            print(f"Auto-stopping after {duration} seconds.")
            threading.Timer(duration, lambda: pipeline.send_event(Gst.Event.new_eos())).start()
    # Wait for 'q' in main thread
    q_thread = threading.Thread(target=wait_for_q, daemon=True)
    q_thread.start()
    try:
        mainloop.run()
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        pipeline.set_state(Gst.State.NULL)
        print("Pipeline stopped.")

if __name__ == "__main__":
    record_with_preview(
        device="/dev/video9",
        outfile="scene1.mp4",
        duration=None,        # Or e.g. 60 for 1 minute auto-stop
        live_view=False,       # Set False to record only, no preview
        jetson=True          # Set True for Jetson hardware encoder
    )
