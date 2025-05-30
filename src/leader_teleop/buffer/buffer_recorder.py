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
        output_dir="recordings",
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

        self.session_counter += 1
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_name = f"{self.session_id}_{self.session_counter:03d}_{timestamp}"
        session_path = os.path.join(self.output_dir, session_name)
        os.makedirs(session_path, exist_ok=True)

        # Start camera pipelines if available
        if self.camera_pipeline_manager:
            self.camera_pipeline_manager.start_recording(output_dir=session_path)

        # Start data buffer logging
        output_file = os.path.join(session_path, "data.csv")
        self.stop_event = threading.Event()
        self.thread = BufferReaderThread(
            self.sync_buffer,
            output_file,
            self.stop_event,
            poll_frequency=self.poll_frequency,
        )
        self.thread.start()
        print(f"📁 Started recording session: {session_path}")

    def stop_recording(self):
        if self.thread and self.thread.is_alive():
            self.stop_event.set()
            self.thread.join()
            print("🛑 Buffer recording stopped.")
        else:
            print("⚠️ No buffer recording in progress.")

        if self.camera_pipeline_manager:
            self.camera_pipeline_manager.stop_recording()
            print("🎥 Camera recording stopped.")
