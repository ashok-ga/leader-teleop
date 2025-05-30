import os
import threading
import time
from datetime import datetime

from blessed import Terminal
from leader_teleop.dynamixel_reader import reader_thread
from leader_teleop.piper import (
    NUM_JOINTS,
    PIPER_ACCEL,
    PIPER_SPEED,
    piper_reader_thread,
    piper_sender_thread,
)
from gripper import SERVO_ID, gripper_thread
from leader_teleop.camera.camera import (
    CameraPipelineManager,
)  # Optional: can keep it if it does something else
from leader_teleop.buffer.data_sync_buffer import DataSyncBuffer
from leader_teleop.buffer.buffer_recorder import (
    BufferRecorder,
)  # <- Your BufferRecorder class

# ───────── shared state & stop event ─────────
event_stop = threading.Event()


def main():

    # Sensors writing to DataSyncBuffer
    data_buffers = [
        "ServoCmd",
        "ServoPos",
        "diffs",
        "eef_pose",
        "scene_camera_bottom",
        "scene_camera_top",
        "wrist_camera_right",
    ]  # adjust to match actual names used in buffer

    data_sync_buffer = DataSyncBuffer(sensors=data_buffers)

    camera_pipeline_manager = None
    camera_pipeline_manager = CameraPipelineManager(data_sync_buffer)

    threads = [
        threading.Thread(
            target=reader_thread,
            args=(event_stop, data_sync_buffer),
            daemon=False,
        ),
        threading.Thread(
            target=piper_reader_thread,
            args=(event_stop, data_sync_buffer),
            daemon=True,
        ),
        threading.Thread(
            target=piper_sender_thread,
            args=(event_stop, data_sync_buffer),
            daemon=True,
        ),
        threading.Thread(
            target=gripper_thread,
            args=(event_stop, data_sync_buffer),
            daemon=True,
        ),
    ]

    for t in threads:
        t.start()

    term = Terminal()
    try:
        session_name = input("Enter session name: ")
        buffer_recorder = BufferRecorder(
            session_name, data_sync_buffer, camera_pipeline_manager
        )
        with term.cbreak():
            while True:
                print("Press 's' to start recording, 't' to end session.")

                key = term.inkey(timeout=None).lower()
                if key == "s":
                    print("Starting recording… press q to stop")

                    buffer_recorder.start_recording()

                    while term.inkey(timeout=None).lower() != "q":
                        pass

                    buffer_recorder.stop_recording()
                    print("Trial stopped.")

                elif key == "t":
                    print("✱ Ending session.")
                    break
    except KeyboardInterrupt:
        print("\nInterrupted; ending.")

    event_stop.set()
    for t in threads:
        t.join()

    print("✅ Goodbye.")


if __name__ == "__main__":
    main()
