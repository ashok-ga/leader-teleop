#!/usr/bin/env python3
# -*- coding: utf8 -*-

import time
import yaml
import math
import threading
import serial  # RS485 gripper control
import random
from collections import defaultdict
from dynamixel_sdk import *
from piper_sdk import C_PiperInterface_V2

CONFIG_FILE = "robot_config.yaml"
PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 1000000
SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200
POSITION_KEY = 'present_position'

# Default speed/accel constants
DEFAULT_SPEED = 1000
DEFAULT_ACCEL = 500
DEG_FACTOR = 57295.7795  # Piper units per radian

# RS485 gripper servo parameters
SERVO_ID = 1
POSITION_1 = 1270
POSITION_2 = 2276
SERVO_SPEED = 7500
SERVO_ACCEL = 0


def calculate_checksum(packet):
    checksum = sum(packet[2:]) & 0xFF
    return (~checksum) & 0xFF


def build_move_command(servo_id, position, speed, accel):
    packet = [
        0xFF, 0xFF, servo_id, 0x08, 0x03, 0x2A,
        position & 0xFF, (position >> 8) & 0xFF,
        speed & 0xFF, (speed >> 8) & 0xFF,
        accel, 0x00
    ]
    packet[-1] = calculate_checksum(packet)
    return bytes(packet)


def build_position_query_packet(servo_id):
    packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x38, 0x02]
    packet.append(calculate_checksum(packet))
    return bytes(packet)


def read_servo_position(ser, servo_id):
    query_packet = build_position_query_packet(servo_id)
    ser.flushInput()
    ser.write(query_packet)
    timeout = time.time() + 0.005
    response = []
    while time.time() < timeout:
        if ser.in_waiting:
            response.extend(ser.read(ser.in_waiting))
    for i in range(len(response)-7):
        if response[i] == 0xFF and response[i+1] == 0xFF and response[i+2] == servo_id:
            return response[i+5] | (response[i+6] << 8)
    return -1


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
            # include gripper joint if defined in config
            self.joint_groups[joint['motor_type']].append(joint)
        for mtype, joints in self.joint_groups.items():
            table = self.motor_tables[mtype]
            reader = GroupSyncRead(
                self.port_handler, self.packet_handler,
                table[POSITION_KEY], table['position_bytes']
            )
            for j in joints:
                reader.addParam(j['id'])
            self.pos_readers[mtype] = reader

    def read_all(self):
        angles = {}
        for mtype, joints in self.joint_groups.items():
            table = self.motor_tables[mtype]
            reader = self.pos_readers[mtype]
            if not reader.txRxPacket():
                for j in joints:
                    raw = reader.getData(j['id'], table[POSITION_KEY], table['position_bytes'])
                    pos = int.from_bytes(raw.to_bytes(table['position_bytes'], 'little'), 'little')
                    if mtype in ['XL430','XM430']:
                        deg = (pos / 4095.0) * 360.0
                    elif mtype == 'XL320':
                        deg = (pos / 1023.0) * 300.0
                    else:
                        deg = pos
                    angles[j['name']] = math.radians(deg)
        return angles

    def shutdown(self):
        self.port_handler.closePort()


def reader_thread(shared_state, lock, stop_flag, config):
    reader = SyncReader(config)
    initial = reader.read_all()
    try:
        while not stop_flag['stop']:
            current = reader.read_all()
            diffs = {name: current.get(name, initial.get(name, 0.0)) - initial.get(name, 0.0)
                     for name in initial}
            with lock:
                shared_state['diffs'] = diffs
            time.sleep(0.01)
    finally:
        reader.shutdown()


def sender_thread(shared_state, lock, stop_flag, piper):
    names = [f"joint_{i}" for i in range(1, 7)]
    try:
        while not stop_flag['stop']:
            with lock:
                diffs = shared_state.get('diffs', {}) or {}
            ordered = [diffs.get(name, 0.0) for name in names]
            try:
                piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            except Exception as e:
                print(f"❌ MotionCtrl_2 error: {e}")
            values = []
            for idx, delta_rad in enumerate(ordered):
                if idx in (2,4):
                    delta_rad = -delta_rad
                cmd = int(delta_rad * DEG_FACTOR + 0.5)
                values.append(cmd)
            try:
                piper.JointCtrl(*values)
            except Exception as e:
                print(f"❌ JointCtrl error: {e}")
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass


def gripper_thread(shared_state, lock, stop_flag, ser):
    try:
        while not stop_flag['stop']:
            with lock:
                # Ensure your config names the gripper joint appropriately
                diff = shared_state.get('diffs', {}).get('gripper', 0.0)
            diff_clamped = max(0.0, min(diff, 1.0))  # map 0–1 rad
            position = int(POSITION_1 + diff_clamped * (POSITION_2 - POSITION_1))
            print(position)
            ser.write(build_move_command(SERVO_ID, position, SERVO_SPEED, SERVO_ACCEL))
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


def load_config(path=CONFIG_FILE):
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    config = load_config()

    # Initialize Piper
    piper = C_PiperInterface_V2('can0')
    piper.ConnectPort()
    while not piper.EnablePiper():
        time.sleep(0.01)
    for i in range(1, 7):
        piper.JointMaxAccConfig(i, DEFAULT_ACCEL)
        piper.MotorMaxSpdSet(i, DEFAULT_SPEED)
        time.sleep(0.02)

    # Initialize RS485 serial for gripper
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud for gripper control.")

    lock = threading.Lock()
    shared_state = {}
    stop_flag = {'stop': False}
    t_reader = threading.Thread(target=reader_thread, args=(shared_state, lock, stop_flag, config), daemon=True)
    t_sender = threading.Thread(target=sender_thread, args=(shared_state, lock, stop_flag, piper), daemon=True)
    t_gripper = threading.Thread(target=gripper_thread, args=(shared_state, lock, stop_flag, ser), daemon=True)

    t_reader.start()
    t_sender.start()
    t_gripper.start()

    try:
        t_reader.join()
        t_sender.join()
        t_gripper.join()
    except KeyboardInterrupt:
        stop_flag['stop'] = True
        t_reader.join()
        t_sender.join()
        t_gripper.join()


if __name__ == '__main__':
    main()
