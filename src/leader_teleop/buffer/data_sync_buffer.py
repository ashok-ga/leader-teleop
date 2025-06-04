from collections import deque
from logging import warn, warning
import threading
import time


class Buffer:
    def __init__(self, maxlen=100):
        self.data = deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def add(self, value, timestamp=None):
        """
        Adds a value to the buffer with the current timestamp if not provided.
        :param value: the value to store
        :param timestamp: optional custom timestamp
        """
        assert timestamp is None
        ts = timestamp if timestamp is not None else time.time()
        with self.lock:
            self.data.append((ts, value))

    def __len__(self):
        with self.lock:
            return len(self.data)

    def __getitem__(self, index):
        with self.lock:
            return self.data[index]

    def __iter__(self):
        with self.lock:
            return iter(list(self.data))  # return a copy to prevent race conditions


class DataSyncBuffer:
    # We use a tolerance of 34ms since the cameras are running at 30FPS
    def __init__(self, sensors, maxlen=100, tolerance=0.050):
        self.buffers = {s: Buffer(maxlen=maxlen) for s in sensors}
        self.tolerance = tolerance
        self.lock = threading.Lock()

    def get_synced(self, sensors=None):
        with self.lock:
            target_sensors = sensors if sensors is not None else self.buffers.keys()

            # for sensor in target_sensors:
            #     print(f"Checking sensor: {sensor}, {len(self.buffers[sensor])} entries")

            # Ensure all requested buffers have at least one entry
            if not all(len(self.buffers[s]) > 0 for s in target_sensors):
                return None

            # Use the latest timestamp from the least-filled buffer as sync target
            base_ts = max(self.buffers[s][-1][0] for s in target_sensors)
            base_sensor = None
            for sensor in target_sensors:
                if self.buffers[sensor][-1][0] == base_ts:
                    base_sensor = sensor
                    break

            base_ts = max(self.buffers[s][-1][0] for s in target_sensors)
            result = {}
            for sensor in target_sensors:
                buf = self.buffers[sensor]
                closest = min(buf, key=lambda x: abs(x[0] - base_ts))
                # print(
                #     f"Sensor: {sensor}, Closest timestamp: {closest[0]}, Base timestamp: {base_ts}"
                # )
                if abs(closest[0] - base_ts) > self.tolerance:
                    warning(
                        f"Breach of tolerance for {sensor}: {abs(closest[0] - base_ts)} > {self.tolerance} wrt {base_sensor} at {base_ts}. "
                        "This may lead to desynchronization."
                    )
                    # return None
                result[sensor] = closest[1]
            return result

    def get_buffer(self, sensor):
        return self.buffers[sensor]

    def clear(self):
        for buf in self.buffers.values():
            buf.data.clear()
