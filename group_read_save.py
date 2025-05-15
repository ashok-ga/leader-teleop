import yaml
import time
import csv
import math
from collections import defaultdict
from multiprocessing import Process, Queue
from dynamixel_sdk import *
from commands import DynamixelCommands  # Your command abstraction

CONFIG_FILE = "robot_config.yaml"
PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 1000000
POSITION_KEY = 'present_position'
CSV_FILE = "joint_angle_deltas.csv"

class SyncReader:
    def __init__(self, config):
        self.config = config
        self.port_handler = PortHandler(PORT_NAME)
        self.packet_handler = PacketHandler(2.0)

        if not self.port_handler.openPort():
            raise RuntimeError("❌ Failed to open port")
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise RuntimeError("❌ Failed to set baudrate")

        self.motor_tables = config['motor_tables']
        self.joint_groups = defaultdict(list)
        self.pos_readers = {}

        for joint in config['robot']['joints']:
            motor_type = joint['motor_type']
            self.joint_groups[motor_type].append(joint)

        for motor_type, joints in self.joint_groups.items():
            table = self.motor_tables[motor_type]
            reader = GroupSyncRead(
                self.port_handler, self.packet_handler,
                table[POSITION_KEY], table['position_bytes']
            )
            for joint in joints:
                reader.addParam(joint['id'])
            self.pos_readers[motor_type] = reader

    def read_all(self):
        all_results = []
        for motor_type, joints in self.joint_groups.items():
            table = self.motor_tables[motor_type]
            reader = self.pos_readers[motor_type]

            if reader.txRxPacket():
                print(f"❌ SyncRead failed for {motor_type}")
                continue

            for joint in joints:
                dxl_id = joint['id']
                if reader.isAvailable(dxl_id, table[POSITION_KEY], table['position_bytes']):
                    raw = reader.getData(dxl_id, table[POSITION_KEY], table['position_bytes'])
                    pos = int.from_bytes(raw.to_bytes(table['position_bytes'], 'little'), 'little')
                else:
                    pos = None

                if pos is not None:
                    if motor_type in ['XL430', 'XM430']:
                        angle_deg = (pos / 4095.0) * 360.0
                    elif motor_type == 'XL320':
                        angle_deg = (pos / 1023.0) * 300.0
                    else:
                        angle_deg = pos
                    angle_rad = math.radians(angle_deg)
                else:
                    angle_rad = None

                all_results.append((joint['name'], pos, angle_rad))
        return all_results

    def shutdown(self):
        self.port_handler.closePort()


def load_config():
    with open(CONFIG_FILE, 'r') as f:
        return yaml.safe_load(f)


def reader_worker(queue: Queue):
    config = load_config()
    reader = SyncReader(config)
    joints = [joint['name'] for joint in config['robot']['joints']]

    # Capture initial positions
    initial_angles = {}
    while not initial_angles:
        init_results = reader.read_all()
        initial_angles = {
            name: angle for name, _, angle in init_results if angle is not None
        }

    # Write CSV header
    with open(CSV_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(joints)

    try:
        while True:
            if not queue.empty():
                cmd = queue.get()
                if cmd == 'exit':
                    break

            results = reader.read_all()
            current_angles = {name: angle for name, _, angle in results}

            # Compute differences from initial position
            deltas = [
                current_angles.get(name, float('nan')) - initial_angles.get(name, 0.0)
                for name in joints
            ]

            print(deltas)

            # Append to CSV
            with open(CSV_FILE, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(deltas)

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
        queue.put('exit')
        p.join()


if __name__ == '__main__':
    main()
