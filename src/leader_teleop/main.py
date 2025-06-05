import os
import threading
import time
from datetime import datetime

from blessed import Terminal
from leader_teleop.dynamixel_reader import (
    dynamixel_init,
    dynamixel_shutdown,
)
from leader_teleop.piper import (
    NUM_JOINTS,
    PIPER_ACCEL,
    PIPER_SPEED,
    piper_init,
    piper_shutdown,
)
from gripper import SERVO_ID, gripper_init, gripper_shutdown
from leader_teleop.camera.camera import (
    CameraPipelineManager,
)  # Optional: can keep it if it does something else
from leader_teleop.buffer.data_sync_buffer import DataSyncBuffer
from leader_teleop.buffer.buffer_recorder import (
    BufferRecorder,
)  # <- Your BufferRecorder class


def main():

    # Sensors writing to DataSyncBuffer
    data_buffers = [
        "right_servo_cmds",
        "right_servo_pos",
        "left_servo_cmds",
        "left_servo_pos",
        "right_diffs",
        "left_diffs",
        "right_eef_pose",
        "left_eef_pose",
        "scene_camera_bottom",
        "scene_camera_top",
        "wrist_camera_right",
        "wrist_camera_left",
    ]  # adjust to match actual names used in buffer

    session_name = input("Enter session name: ")
    outptut_dir = os.path.join(
        "recordings", f"{session_name}_{datetime.now().strftime("%Y%m%d_%H%M%S")}"
    )

    data_sync_buffer = DataSyncBuffer(sensors=data_buffers)

    camera_pipeline_manager = None
    camera_pipeline_manager = CameraPipelineManager(
        data_sync_buffer,
        output_dir=outptut_dir,
    )

    pr, pl = piper_init(data_sync_buffer)
    dr, dl = dynamixel_init(data_sync_buffer)
    gr, gl = gripper_init(data_sync_buffer)

    term = Terminal()
    try:
        buffer_recorder = BufferRecorder(
            session_name,
            data_sync_buffer,
            camera_pipeline_manager,
            output_dir=outptut_dir,
            poll_frequency=20.0,
        )
        with term.cbreak():
            while True:
                print("Press 'space' to start recording, 't' to end session.")

                key = term.inkey(timeout=None).lower()
                if key == " ":
                    print("Starting recording… press q to stop")

                    buffer_recorder.start_recording()

                    while term.inkey(timeout=None).lower() != " ":
                        pass

                    buffer_recorder.stop_recording()
                    print("Trial stopped.")

                elif key == "t":
                    print("✱ Ending session.")
                    break
    except KeyboardInterrupt:
        print("\nInterrupted; ending.")

    camera_pipeline_manager.shutdown()
    piper_shutdown(pr, pl)
    dynamixel_shutdown(dr, dl)
    gripper_shutdown(gr, gl)

    print("✅ Goodbye.")


if __name__ == "__main__":
    main()
