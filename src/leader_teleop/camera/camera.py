# ───────── camera thread ─────────
import csv
from datetime import datetime
import time
from typing import Dict, Tuple

from leader_teleop.camera.gsn import GstreamerCameraRecorder
from leader_teleop.config.utils import device_config


from pathlib import Path
import gi
from gi.repository import Gst, GLib

import multiprocessing as mp

gi.require_version("Gst", "1.0")

Gst.init(None)


class CameraPipelineManager:
    def __init__(self, data_sync_buffer, output_dir="recordings"):
        self.data_sync_buffer = data_sync_buffer
        self.camera_config = device_config["cameras"]
        self.recorders: Dict[Tuple, GstreamerCameraRecorder] = (
            {}
        )  # (cam_type, alias) -> GstreamerCameraRecorder
        self.init_pipelines(output_dir=output_dir)

    def init_pipelines(self, output_dir):
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        for cam_type, cameras in self.camera_config.items():
            for alias, cam in cameras.items():
                output_pattern = output_dir / f"{alias}_%d.mp4"
                recorder = GstreamerCameraRecorder(
                    sync_buffer=self.data_sync_buffer.get_buffer(alias),
                    output_pattern=output_pattern,
                    device=cam["device"],
                    width=cam["width"],
                    height=cam["height"],
                    caps=cam["caps"],
                    verbose=False,
                )
                self.recorders[(cam_type, alias)] = recorder

        print(f"📹 Starting camera recording for {len(self.recorders)} cameras.")
        time.sleep(3)  # Allow time for pipelines to initialize

        print(f"📂 Camera pipelines initialized. Output directory: {output_dir}")

    def start_recording(self):
        if not self.recorders:
            print("No camera recorders initialized. Please call init_pipelines first.")
            return

        for (cam_type, alias), recorder in self.recorders.items():
            if not recorder.is_recording():
                print(f"Starting recording for {alias} ({cam_type})")
                recorder.start_recording()
            else:
                print(f"Recorder for {alias} ({cam_type}) is already running.")

    def stop_recording(self):
        for (cam_type, alias), recorder in self.recorders.items():
            if hasattr(recorder, "shutdown"):
                recorder.stop_recording()

    def shutdown(self):
        for (cam_type, alias), recorder in self.recorders.items():
            if hasattr(recorder, "shutdown"):
                recorder.shutdown()
        print("All camera pipelines shut down.")


def dry_run():
    from leader_teleop.buffer.data_sync_buffer import DataSyncBuffer

    cameras = device_config["cameras"]
    cam_buffers = []
    for cam_type, camera_list in cameras.items():
        for alias, cam in camera_list.items():
            print(
                f"Camera {alias} ({cam_type}): {cam['device']} - {cam['width']}x{cam['height']} - {cam['caps']}"
            )
            cam_buffers.append(alias)

    data_sync_buffer = DataSyncBuffer(cam_buffers)

    output_dir = "temp_recording"
    camera_pipeline_manager = CameraPipelineManager(
        data_sync_buffer, output_dir=output_dir
    )

    # Start recording to a temporary directory
    camera_pipeline_manager.start_recording()
    time.sleep(5)  # Simulate some recording time

    for (cam_type, alias), recorder in camera_pipeline_manager.recorders.items():
        buffer_data = list(data_sync_buffer.get_buffer(alias).data)[-10:]
        time_diffs = [
            buffer_data[i + 1][0] - buffer_data[i][0]
            for i in range(len(buffer_data) - 1)
        ]
        print(alias, "Time differences between consecutive elements:", time_diffs)

    # Stop recording
    camera_pipeline_manager.stop_recording()

    camera_pipeline_manager.shutdown()

    print(f"Recording stopped. Files saved in: {output_dir}")


if __name__ == "__main__":
    dry_run()

# def camera_thread(state, state_lock, stop_flag, session):
#     # grab once parameters
#     stereo = np.ascontiguousarray(np.empty((h, w * 2, 4), np.uint8))

#     cf = open(f"{out_dir}/data.csv", "w", newline="")
#     wr = csv.writer(cf)
#     hdr = ["Timestamp", "ServoCmd", "ServoPos", "x", "y", "z", "qx", "qy", "qz"]
#     wr.writerow(hdr)
#     pipeline.start_recording(f"{out_dir}/video.mp4")
#     try:
#         while not stop["stop"]:
#             if z.grab(rt) == sl.ERROR_CODE.SUCCESS:
#                 t = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
#                 with state_lock:
#                     pa = state.get("eef_pose", [0] * NUM_JOINTS)
#                     gd = state.get("diffs", {}).get("gripper", 0.0)
#                 pos = float(min(max(gd, GD_LOW), GD_HIGH) / (GD_HIGH - GD_LOW))
#                 sr = read_servo_position(ser, SERVO_ID)
#                 sn = min(max((sr - pos_min) / (pos_max - pos_min), 0.0), 1.0)
#                 wr.writerow([t, pos, sn] + list(pa.values()))
#                 pipeline.push_frame(stereo)
#     finally:
#         pipeline.stop_recording()
#         cf.close()
#         print(f"Recorded files: {len(os.listdir(out_dir))}")
