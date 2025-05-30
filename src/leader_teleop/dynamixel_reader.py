from collections import defaultdict
import time
from leader_teleop.config.utils import device_config, config
from piper import NUM_JOINTS
import math
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead


POSITION_KEY = "present_position"
DXL_BAUD = 4000000


# ───────── SyncReader for Dynamixel ─────────
class SyncReader:
    def __init__(self):
        dxl_port = device_config["dxl_device"]
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
        # store reader plus address/length
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

        # ───── moving average buffers ─────
        # Only for joints 1-6 (not gripper)
        self.filt_bufs = {}
        self.filt_idx = {}
        self.filt_count = {}
        self.filt_win = 10
        for mtype, info in self.readers.items():
            for j in info["joints"]:
                if j["name"] != "gripper":
                    self.filt_bufs[j["name"]] = [0.0] * self.filt_win
                    self.filt_idx[j["name"]] = 0
                    self.filt_count[j["name"]] = 0

    def read_all(self):
        out = {}
        for info in self.readers.values():
            reader = info["reader"]
            if reader.txRxPacket():
                continue
            addr = info["address"]
            length = info["length"]
            for j in info["joints"]:
                raw = reader.getData(j["id"], addr, length)
                if j["name"] == "gripper":
                    val = raw
                else:
                    scale = 4095.0 if j["motor_type"] in ["XL430", "XM430"] else 1023.0
                    rng = 360.0 if j["motor_type"] in ["XL430", "XM430"] else 300.0
                    val = (raw / scale) * rng
                val_rad = math.radians(val)
                # Apply moving average only for joints 1–6 (not gripper)
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
                    out[j["name"]] = val_rad if j["name"] != "gripper" else val
        return out

    def shutdown(self):
        self.port.closePort()


def reader_thread(event_stop, data_sync_buffer, poll_frequency=333.0):
    rd = SyncReader()
    base = rd.read_all()
    print("Base positions:", base)
    interval = 1.0 / poll_frequency

    while not event_stop.is_set():
        cur = rd.read_all()
        diffs = {k: cur.get(k, base[k]) - base[k] for k in base}
        data_sync_buffer.get_buffer("diffs").add(diffs)
        time.sleep(interval)

    rd.shutdown()
