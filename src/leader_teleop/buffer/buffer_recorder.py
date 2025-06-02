import os
import threading
from datetime import datetime
import time

from leader_teleop.camera.camera import CameraPipelineManager
from .buffer_reader import BufferReaderThread


class BufferRecorder:
    def __init__(
        self,
        session_id,
        sync_buffer,
        camera_pipeline_manager: CameraPipelineManager,
        output_dir,
        poll_frequency=20.0,
    ):
        self.session_id = session_id
        self.sync_buffer = sync_buffer
        self.output_dir = output_dir
        self.poll_frequency = poll_frequency
        self.camera_pipeline_manager = camera_pipeline_manager

        self.session_counter = 0
        self.thread = None
        self.stop_event = None
        os.makedirs(output_dir, exist_ok=True)

    def start_recording(self):
        if self.thread and self.thread.is_alive():
            print("⚠️ Recording already in progress.")
            return

        # Start data buffer logging
        output_file = os.path.join(self.output_dir, f"data_{self.session_counter}.csv")

        # Start camera pipelines if available
        if self.camera_pipeline_manager:
            self.camera_pipeline_manager.start_recording()

        # time.sleep(0.3)

        self.stop_event = threading.Event()
        self.thread = BufferReaderThread(
            self.sync_buffer,
            output_file,
            self.stop_event,
            poll_frequency=self.poll_frequency,
        )
        self.thread.start()
        print(f"📁 Started recording session: {self.session_counter}")

    def stop_recording(self):
        if self.camera_pipeline_manager:
            self.camera_pipeline_manager.stop_recording()
            print("🎥 Camera recording stopped.")

        if self.thread and self.thread.is_alive():
            self.stop_event.set()
            self.thread.join()
            print("🛑 Buffer recording stopped.")
        else:
            print("⚠️ No buffer recording in progress.")

        self.session_counter += 1
