#!/usr/bin/env python3
# -*- coding: utf8 -*-

import time
import yaml
import math
import threading
from collections import defaultdict
from dynamixel_sdk import *
from piper_sdk import C_PiperInterface_V2

CONFIG_FILE = "robot_config.yaml"
PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 4000000
POSITION_KEY = 'present_position'

# Default speed/accel constants
DEFAULT_SPEED = 3000
DEFAULT_ACCEL = 500

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
        # set up sync reads for all joint types
        for joint in config['robot']['joints']:
            if joint['id'] >= 6:
                continue
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
        """Reads all configured joints, returns dict of joint_name -> angle_rad"""
        angles = {}
        for mtype, joints in self.joint_groups.items():
            table = self.motor_tables[mtype]
            reader = self.pos_readers[mtype]
            if reader.txRxPacket():
                print(f"❌ SyncRead failed for {mtype}")
                continue
            for j in joints:
                jid = j['id']
                if not reader.isAvailable(jid, table[POSITION_KEY], table['position_bytes']):
                    continue
                raw = reader.getData(jid, table[POSITION_KEY], table['position_bytes'])
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
    except KeyboardInterrupt:
        pass
    finally:
        reader.shutdown()

def sender_thread(shared_state, lock, stop_flag, piper):
    """
    Reads diffs from shared_state and sends them directly as radian diff commands.
    Assumes six joints corresponding to diffs["joint_1"]...diffs["joint_6"].
    JointCtrl will be called with float radians values.
    """
    names = [f"joint_{i}" for i in range(1, 7)]
    try:
        while not stop_flag['stop']:
            # Safely retrieve diffs
            with lock:
                diffs = shared_state.get('diffs', {}) or {}

            # Ordered raw radian diffs
            ordered = [diffs.get(name, 0.0) for name in names]
            print(ordered)

            # Motion control
            try:
                piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            except Exception as e:
                print(f"❌ MotionCtrl_2 error: {e}")

                        # Build and send integer commands from radian diffs using DEG_FACTOR
            DEG_FACTOR = 57295.7795  # Piper units per radian
            # values = int(diff * DEG_FACTOR) per joint, invert joints 3 & 5
            values = []
            for idx, delta_rad in enumerate(ordered):
                if idx in (2, 4):  # invert for joint_3 and joint_5
                    delta_rad = -delta_rad
                cmd = int(delta_rad * DEG_FACTOR + 0.5)
                values.append(cmd)

            print(f"Sending counts: {values}")
            try:
                piper.JointCtrl(*values)
            except Exception as e:
                print(f"❌ JointCtrl error: {e}")

            time.sleep(0.005)
    except KeyboardInterrupt:
        pass

def load_config(path=CONFIG_FILE):
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('config', nargs='?', default=CONFIG_FILE)
    args = parser.parse_args()
    config = load_config(args.config)

    # init Piper interface
    piper = C_PiperInterface_V2('can0')
    piper.ConnectPort()
    while not piper.EnablePiper():
        time.sleep(0.01)

    # configure speed/accel for joints 1-6
    for i in range(1, 7):
        piper.JointMaxAccConfig(i, DEFAULT_ACCEL)
        piper.MotorMaxSpdSet(i, DEFAULT_SPEED)
        time.sleep(0.02)

    # start threads
    lock = threading.Lock()
    shared_state = {}
    stop_flag = {'stop': False}
    t_reader = threading.Thread(
        target=reader_thread,
        args=(shared_state, lock, stop_flag, config),
        daemon=True
    )
    t_sender = threading.Thread(
        target=sender_thread,
        args=(shared_state, lock, stop_flag, piper),
        daemon=True
    )
    t_reader.start()
    t_sender.start()
    try:
        t_reader.join()
        t_sender.join()
    except KeyboardInterrupt:
        stop_flag['stop'] = True
        t_reader.join()
        t_sender.join()

if __name__=='__main__':
    main()
