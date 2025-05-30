import yaml
import time
from collections import defaultdict
from multiprocessing import Process, Queue
from dynamixel_sdk import *
from commands import DynamixelCommands  # Your command abstraction

CONFIG_FILE = "/home/nvidia/leader-teleop/src/leader_teleop/config/robot_config.yaml"
PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 4000000
POSITION_KEY = "present_position"


class SyncReader:
    def __init__(self, config):
        self.config = config
        self.port_handler = PortHandler(PORT_NAME)
        self.packet_handler = PacketHandler(2.0)

        if not self.port_handler.openPort():
            raise RuntimeError("❌ Failed to open port")
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise RuntimeError("❌ Failed to set baudrate")

        self.motor_tables = config["motor_tables"]
        self.joint_groups = defaultdict(list)
        self.pos_readers = {}

        # Build joint groups and SyncRead handlers
        for joint in config["robot"]["joints"]:
            motor_type = joint["motor_type"]
            self.joint_groups[motor_type].append(joint)

        for motor_type, joints in self.joint_groups.items():
            table = self.motor_tables[motor_type]
            reader = GroupSyncRead(
                self.port_handler,
                self.packet_handler,
                table[POSITION_KEY],
                table["position_bytes"],
            )
            for joint in joints:
                reader.addParam(joint["id"])
            self.pos_readers[motor_type] = reader

    def read_all(self):
        all_results = []
        for motor_type, joints in self.joint_groups.items():
            table = self.motor_tables[motor_type]
            reader = self.pos_readers[motor_type]

            # Read all at once
            if reader.txRxPacket():
                print(f"❌ SyncRead failed for {motor_type}")
                continue

            for joint in joints:
                dxl_id = joint["id"]
                raw = reader.getData(
                    dxl_id, table[POSITION_KEY], table["position_bytes"]
                )
                pos = int.from_bytes(
                    raw.to_bytes(table["position_bytes"], "little"), "little"
                )

                # Convert to angle
                if pos is not None:
                    if motor_type in ["XL430", "XM430"]:
                        angle = (pos / 4095.0) * 360.0
                    elif motor_type == "XL320":
                        angle = (pos / 1023.0) * 300.0
                    else:
                        angle = pos  # Raw fallback
                else:
                    angle = None

                all_results.append((joint["name"], pos, angle))
        return all_results

    def shutdown(self):
        self.port_handler.closePort()


def load_config():
    with open(CONFIG_FILE, "r") as f:
        return yaml.safe_load(f)


def reader_worker(queue: Queue):
    config = load_config()
    reader = SyncReader(config)
    frame_count = 0
    start_time = time.time()

    try:
        while True:
            if not queue.empty():
                cmd = queue.get()
                if cmd == "exit":
                    break

            # Read and print
            results = reader.read_all()
            joint_state = {name: angle for name, _, angle in results}
            print(joint_state)
            # No need to sleep, let CAN bus pace itself
    except KeyboardInterrupt:
        print("🛑 Reader interrupted")
    finally:
        reader.shutdown()


def main():
    queue = Queue()
    p = Process(target=reader_worker, args=(queue,))
    p.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("🛑 Stopping reader...")
        queue.put("exit")
        p.join()


if __name__ == "__main__":
    main()
