import math
import threading
import time
from collections import defaultdict

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead

from leader_teleop.config.utils import device_config, config
from piper import NUM_JOINTS

POSITION_KEY = "present_position"
DXL_BAUD = 4000000


class DynamixelReader:
    def __init__(self, diffs, dxl_port, filt_win=10, poll_frequency=100.0):
        self.diffs = diffs
        self.poll_frequency = poll_frequency
        self._stop_event = threading.Event()
        self._thread = None
        self.filt_win = filt_win
        self._setup(dxl_port)
        assert diffs is not None, "DataSyncBuffer must be provided"

    def _setup(self, dxl_port):
        print(f"Opening Dynamixel port: {dxl_port}")
        self.port = PortHandler(dxl_port)
        self.handler = PacketHandler(2.0)
        self.port.openPort()
        print(f"Setting baud rate: {DXL_BAUD}")
        self.port.setBaudRate(DXL_BAUD)

        tables = config["motor_tables"]
        groups = defaultdict(list)
        for j in config["robot"]["joints"]:
            if j["id"] <= NUM_JOINTS:
                groups[j["motor_type"]].append(j)
        self.readers = {}
        for mtype, joints in groups.items():
            tbl = tables[mtype]
            addr = tbl[POSITION_KEY]
            length = tbl["position_bytes"]
            reader = GroupSyncRead(self.port, self.handler, addr, length)
            for j in joints:
                reader.addParam(j["id"])
            self.readers[mtype] = {
                "reader": reader,
                "address": addr,
                "length": length,
                "joints": joints,
            }
        self.filt_bufs = {}
        self.filt_idx = {}
        self.filt_count = {}
        for mtype, info in self.readers.items():
            for j in info["joints"]:
                if j["name"] != "gripper":
                    self.filt_bufs[j["name"]] = [0.0] * self.filt_win
                    self.filt_idx[j["name"]] = 0
                    self.filt_count[j["name"]] = 0

    def read_all(self):
        out = {}
        for info in self.readers.values():
            reader: GroupSyncRead = info["reader"]
            if reader.txRxPacket():
                continue
            addr = info["address"]
            length = info["length"]
            for j in info["joints"]:
                raw = reader.getData(j["id"], addr, length)
                # print(f" motor {j['name']} raw position: {raw}")

                scale = 4095.0 if j["motor_type"] in ["XL430", "XM430"] else 1023.0
                rng = 360.0 if j["motor_type"] in ["XL430", "XM430"] else 300.0
                val = (raw / scale) * rng
                val_rad = math.radians(val)
                if j["name"] != "gripper":
                    buf = self.filt_bufs[j["name"]]
                    idx = self.filt_idx[j["name"]]
                    count = self.filt_count[j["name"]]
                    buf[idx] = val_rad
                    self.filt_idx[j["name"]] = (idx + 1) % self.filt_win
                    if count < self.filt_win:
                        self.filt_count[j["name"]] = count + 1
                    n = self.filt_count[j["name"]]
                    avg_val = sum(buf[:n]) / n
                    out[j["name"]] = avg_val
                else:
                    out[j["name"]] = val
        return out

    def _reader_loop(self):
        base = self.read_all()
        interval = 1.0 / self.poll_frequency
        while not self._stop_event.is_set():
            cur = self.read_all()
            diffs = {k: cur.get(k, base[k]) - base[k] for k in base}
            self.diffs.add(diffs)
            time.sleep(interval)

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._reader_loop)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
        self.port.closePort()


class MockDynamixelReader(DynamixelReader):
    """
    Mock implementation of DynamixelReader for testing purposes.
    It simulates reading joint positions without actual hardware.
    """

    def __init__(self, diffs, dxl_port, poll_frequency=100.0):
        super().__init__(diffs, dxl_port, poll_frequency=poll_frequency)
        self.mock_data = {f"joint_{i}": 0.0 for i in range(1, NUM_JOINTS + 1)}

    def _setup(self, dxl_port):
        pass

    def _reader_loop(self):
        interval = 1.0 / self.poll_frequency
        while not self._stop_event.is_set():
            # Simulate reading joint positions
            for joint in self.mock_data:
                self.mock_data[joint] += 0.01

            diffs = {k: v for k, v in self.mock_data.items()}
            self.diffs.add(diffs)
            time.sleep(interval)


def dynamixel_init(data_sync_buffer):
    """
    Initialize the DynamixelReader with a DataSyncBuffer.
    """
    right_reader = DynamixelReader(
        diffs=data_sync_buffer.get_buffer("right_diffs"),
        dxl_port=device_config["right_dxl_device"],
        poll_frequency=200.0,
    )
    left_reader = DynamixelReader(
        diffs=data_sync_buffer.get_buffer("left_diffs"),
        dxl_port=device_config["left_dxl_device"],
        poll_frequency=200.0,
    )

    right_reader.start()
    left_reader.start()

    print("Dynamixel readers initialized.")
    return right_reader, left_reader


def dynamixel_shutdown(right_reader, left_reader):
    """
    Shutdown the DynamixelReader instances.
    """
    right_reader.stop()
    left_reader.stop()
    print("Dynamixel readers stopped.")


def dry_run():
    from leader_teleop.buffer.data_sync_buffer import DataSyncBuffer

    data_sync_buffer = DataSyncBuffer(["right_diffs", "left_diffs"])

    right_reader, left_reader = dynamixel_init(data_sync_buffer)

    print("Running DynamixelReader for 5 seconds...")
    time.sleep(5)

    dynamixel_shutdown(right_reader, left_reader)

    # print("Right Diffs:", data_sync_buffer.get_buffer("right_diffs").data)
    # print("Left Diffs:", data_sync_buffer.get_buffer("left_diffs").data)
    print("Dry run completed.")


if __name__ == "__main__":
    dry_run()
