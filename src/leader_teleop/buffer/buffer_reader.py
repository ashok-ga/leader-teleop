import os
import csv
import threading
import time
from datetime import datetime

from leader_teleop.buffer.data_sync_buffer import DataSyncBuffer


class BufferReaderThread(threading.Thread):
    def __init__(
        self, sync_buffer: DataSyncBuffer, output_file, stop_event, poll_frequency=20.0
    ):
        """
        :param poll_frequency: Polling rate in Hz (e.g., 100 Hz means 0.01s interval)
        """
        super().__init__()
        self.sync_buffer = sync_buffer
        self.output_file = output_file
        self.stop_event = stop_event
        self.poll_interval = 1.0 / poll_frequency
        self.initial_poll_interval = self.poll_interval / 2
        self.headers_written = False
        self.buffers_to_write = [
            "right_servo_cmds",
            "right_servo_pos",
            "left_servo_cmds",
            "left_servo_pos",
            "right_eef_pose",
            "left_eef_pose",
            "scene_camera_bottom",
            "scene_camera_top",
            "wrist_camera_right",
            "wrist_camera_left",
        ]

    def run(self):
        self.sync_buffer.clear()

        # Wait for all three cameras to have at least one frame
        required_cams = [buf for buf in self.buffers_to_write if "camera" in buf]
        print("[BufferReaderThread] Waiting for all camera streams...")
        while not self.stop_event.is_set():
            # Assume each buffer is a list of (timestamp, data) tuples
            all_present = all(
                self.sync_buffer.buffers.get(cam)
                and len(self.sync_buffer.buffers[cam]) > 0
                for cam in required_cams
            )
            if all_present:
                print("[BufferReaderThread] All camera streams detected.")
                break
            time.sleep(self.initial_poll_interval)  # Poll at 20Hz while waiting

        start_time = time.time()
        # Start CSV writing after all cameras have at least one frame
        with open(self.output_file, mode="w", newline="") as csvfile:
            writer = csv.writer(csvfile)

            while not self.stop_event.is_set():
                synced_data = self.sync_buffer.get_synced()
                assert synced_data is not None, "No synced data available"

                if synced_data:
                    if not self.headers_written:
                        headers = [
                            "timestamp",
                            "right_servo_cmds",
                            "right_servo_pos",
                            "right_x",
                            "right_y",
                            "right_z",
                            "right_qx",
                            "right_qy",
                            "right_qz",
                            "left_servo_cmds",
                            "left_servo_pos",
                            "left_x",
                            "left_y",
                            "left_z",
                            "left_qx",
                            "left_qy",
                            "left_qz",
                        ]
                        writer.writerow(headers)
                        self.headers_written = True

                    timestamp = max(
                        self.sync_buffer.buffers[s][-1][0] for s in synced_data
                    )
                    row = [timestamp] + [
                        synced_data["right_servo_cmds"],
                        synced_data["right_servo_pos"],
                        *list(synced_data["right_eef_pose"].values()),
                        synced_data["left_servo_cmds"],
                        synced_data["left_servo_pos"],
                        *list(synced_data["left_eef_pose"].values()),
                    ]
                    writer.writerow(row)
                time.sleep(self.poll_interval)

        print(
            f"Finished writing to {self.output_file}, Duration: {time.time() - start_time:.2f} seconds"
        )
