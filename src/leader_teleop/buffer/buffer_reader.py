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
        self.headers_written = False

    def run(self):
        with open(self.output_file, mode="w", newline="") as csvfile:
            writer = csv.writer(csvfile)

            while not self.stop_event.is_set():
                print("here")
                synced_data = self.sync_buffer.get_synced(
                    [
                        "ServoCmd",
                        "ServoPos",
                        "eef_pose",
                        "scene_camera_bottom",
                        "scene_camera_top",
                        "wrist_camera_right",
                    ]
                )

                assert synced_data is not None, "No synced data available"

                # print(f"Synced data: {synced_data}")
                if synced_data:
                    if not self.headers_written:
                        headers = [
                            "timestamp",
                            "ServoCmd",
                            "ServoPos",
                            "x",
                            "y",
                            "z",
                            "qx",
                            "qy",
                            "qz",
                        ]
                        writer.writerow(headers)
                        self.headers_written = True

                    timestamp = max(
                        self.sync_buffer.buffers[s][-1][0] for s in synced_data
                    )
                    row = [timestamp] + [
                        synced_data["ServoCmd"],
                        synced_data["ServoPos"],
                        *list(synced_data["eef_pose"].values()),
                    ]
                    writer.writerow(row)

                time.sleep(self.poll_interval)

        self.sync_buffer.clear()
