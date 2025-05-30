#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
leader-follower capture tool (optimized with joint smoothing, lower CPU usage)
► Smoothing: Moving average window=10 for joints 1–6 (not gripper)
"""

import os
import csv
import threading
import struct
import time
import yaml
import math
from datetime import datetime
from collections import defaultdict, deque

import numpy as np
import pyzed.sl as sl
import serial
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead
from piper_sdk import C_PiperInterface_V2
from leader_teleop.camera.gsn import GstreamerRecorder
from blessed import Terminal

# ───────── constants ─────────
CONFIG_FILE   = "robot_config.yaml"
DXL_PORT      = "/dev/ttyUSB0"
DXL_BAUD      = 4000000
CAN_IFACE     = "can0"
RS485_PORT    = "/dev/ttyUSB1"
RS485_BAUD    = 115200
POSITION_KEY  = "present_position"
SERVO_ID      = 1
PIPER_ACCEL   = 500
PIPER_SPEED   = 3000
DEG_FACTOR    = 57295.7795
NUM_JOINTS    = 6
LOOP_HZ       = 300
# gripper extremes
pos_min, pos_max = 1150, 2200
GRIP_SPEED, GRIP_ACCEL = 7500, 0
GD_LOW = 0
GD_HIGH = 30

def make_grip_pkt(pos: int) -> bytes:
    hdr = bytearray([0xFF,0xFF,SERVO_ID,0x08,0x03,0x2A])
    pkt = hdr + struct.pack('<HHB', pos, GRIP_SPEED, GRIP_ACCEL) + b'\x00'
    pkt[-1] = (~sum(pkt[2:]) & 0xFF)
    return bytes(pkt)

# ───────── SyncReader for Dynamixel ─────────
class SyncReader:
    def __init__(self, config):
        self.port = PortHandler(DXL_PORT)
        self.handler = PacketHandler(2.0)
        self.port.openPort()
        self.port.setBaudRate(DXL_BAUD)
        tables = config['motor_tables']
        groups = defaultdict(list)
        for j in config['robot']['joints']:
            if j['id'] <= NUM_JOINTS + 1:  # gripper might be joint 7
                groups[j['motor_type']].append(j)
        # store reader plus address/length
        self.readers = {}
        for mtype, joints in groups.items():
            tbl = tables[mtype]
            addr = tbl[POSITION_KEY]
            length = tbl['position_bytes']
            reader = GroupSyncRead(self.port, self.handler, addr, length)
            for j in joints:
                reader.addParam(j['id'])
            self.readers[mtype] = {
                'reader': reader,
                'address': addr,
                'length': length,
                'joints': joints
            }
        # Use deque for smoothing (O(1) moving average)
        self.smooth_buffers = {j['name']: deque(maxlen=10) for group in groups.values()
                              for j in group if j['name'] != 'gripper'}

    def read_all(self):
        out = {}
        for info in self.readers.values():
            reader = info['reader']
            if reader.txRxPacket():
                continue
            addr = info['address']
            length = info['length']
            for j in info['joints']:
                raw = reader.getData(j['id'], addr, length)
                scale = 4095.0 if j['motor_type'] in ['XL430','XM430'] else 1023.0
                rng = 360.0 if j['motor_type'] in ['XL430','XM430'] else 300.0
                val = (raw / scale) * rng
                if j['name'] == 'gripper':
                    out[j['name']] = val  # No smoothing for gripper
                else:
                    rad = math.radians(val)
                    buf = self.smooth_buffers[j['name']]
                    buf.append(rad)
                    out[j['name']] = float(np.mean(buf))
        return out

    def shutdown(self):
        self.port.closePort()

# ───────── shared state & stop event ─────────
event_stop = threading.Event()
state = {}
state_lock = threading.Lock()

# ───────── threads ─────────
def reader_thread(cfg):
    rd = SyncReader(cfg)
    base = rd.read_all()
    while not event_stop.is_set():
        cur = rd.read_all()
        diffs = {k: cur.get(k, base[k]) - base[k] for k in base}
        with state_lock:
            state['diffs'] = diffs.copy()
        event_stop.wait(0.003)  # ~333 Hz

def piper_reader_thread(piper):
    """Continuously read Piper joint messages and store radians."""
    def _piper_angle_to_rad(angle):
        return angle * math.pi / 180.0 / 1e3

    while not event_stop.is_set():
        msg = piper.GetArmEndPoseMsgs()
        js = msg.end_pose
        eef_pose = {
            "x": js.X_axis / 1e6,
            "y": js.Y_axis / 1e6,
            "z": js.Z_axis / 1e6,
            "qx": _piper_angle_to_rad(js.RX_axis),
            "qy": _piper_angle_to_rad(js.RY_axis),
            "qz": _piper_angle_to_rad(js.RZ_axis),
        }
        with state_lock:
            state["eef_pose"] = eef_pose.copy()
        event_stop.wait(0.005)  # 100 Hz is plenty for end pose

def sender_thread(piper):
    period = 1.0/LOOP_HZ
    next_t = time.perf_counter()
    names = [f'joint_{i}' for i in range(1, NUM_JOINTS+1)]
    while not event_stop.is_set():
        with state_lock:
            diffs = state.get('diffs', {}).copy()
            last_cmds = state.get('last_cmds')
        cmds = []
        for idx,name in enumerate(names):
            d = -diffs.get(name,0.0) if idx in (2,4) else diffs.get(name,0.0)
            cmds.append(int(d * DEG_FACTOR + 0.5))
        if cmds != last_cmds:
            piper.JointCtrl(*cmds)
            with state_lock:
                state['last_cmds'] = cmds
        next_t += period
        event_stop.wait(max(0, next_t - time.perf_counter()))

def gripper_thread(ser):
    while not event_stop.is_set():
        with state_lock:
            gd = state.get('diffs', {}).get('gripper', 0.0)
            last = state.get('last_grip')
        pos = int(pos_min + (min(max(gd, GD_LOW), GD_HIGH)/(GD_HIGH-GD_LOW))*(pos_max - pos_min))
        if pos != last:
            ser.write(make_grip_pkt(pos))
            with state_lock:
                state['last_grip'] = pos
        event_stop.wait(0.001)  # 100 Hz

def calculate_checksum(packet):
    checksum = sum(packet[2:]) & 0xFF
    return (~checksum) & 0xFF

def build_position_query_packet(servo_id):
    packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x38, 0x02]
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    return bytes(packet)

def read_servo_position(ser, servo_id):
    query_packet = build_position_query_packet(servo_id)
    ser.flushInput()
    ser.write(query_packet)

    timeout = time.time() + 0.1  # 100ms timeout
    response = []
    while time.time() < timeout:
        if ser.in_waiting:
            response.extend(ser.read(ser.in_waiting))

    for i in range(len(response) - 7):
        if response[i] == 0xFF and response[i + 1] == 0xFF and response[i + 2] == servo_id:
            position = response[i + 5] | (response[i + 6] << 8)
            return position

    return -1  # Failed to read

def camera_thread(state, lock, stop, ser, name):
    rt = sl.RuntimeParameters()
    L, R = sl.Mat(), sl.Mat()
    info = z.get_camera_information()
    w, h = info.camera_configuration.resolution.width, info.camera_configuration.resolution.height
    stereo = np.ascontiguousarray(np.empty((h, w*2, 4), np.uint8))
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = f'recordings/{name}_{ts}'
    os.makedirs(out_dir, exist_ok=True)
    cf = open(f'{out_dir}/data.csv', 'w', newline='')
    wr = csv.writer(cf)
    hdr = ["Timestamp", "ServoCmd", "ServoPos", "x", "y", "z", "qx", "qy", "qz"]
    wr.writerow(hdr)
    pipeline.start_recording(f'{out_dir}/video.mp4')
    TARGET_FPS = 30
    FRAME_PERIOD = 1.0 / TARGET_FPS

    try:
        while not stop['stop']:
            loop_start = time.perf_counter()
            if z.grab(rt) == sl.ERROR_CODE.SUCCESS:
                z.retrieve_image(L, sl.VIEW.LEFT)
                z.retrieve_image(R, sl.VIEW.RIGHT)
                stereo[:, :w] = L.get_data()
                stereo[:, w:] = R.get_data()
                t = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                with lock:
                    pa = state.get('eef_pose', [0]*NUM_JOINTS)
                    gd = state.get('diffs', {}).get('gripper', 0.0)
                pos = float(min(max(gd, GD_LOW), GD_HIGH)/(GD_HIGH-GD_LOW))
                sr = read_servo_position(ser, SERVO_ID)
                sn = min(max((sr - pos_min)/(pos_max - pos_min), 0.0), 1.0)
                wr.writerow([t, pos, sn] + list(pa.values()))
                pipeline.push_frame(stereo)
            elapsed = time.perf_counter() - loop_start
            to_sleep = FRAME_PERIOD - elapsed
            if to_sleep > 0:
                event_stop.wait(to_sleep)
    finally:
        pipeline.stop_recording()
        cf.close()
        print(f"Recorded files: {len(os.listdir(out_dir))}")

def load_config():
    with open(CONFIG_FILE, 'r') as f:
        return yaml.safe_load(f)

# ───────── main ─────────
def main():
    global z, pipeline, piper
    # initialize camera & pipeline first
    zed_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, camera_fps=30)
    z = sl.Camera()
    assert z.open(zed_params) == sl.ERROR_CODE.SUCCESS, '🔴 ZED open failed'
    pipeline = GstreamerRecorder()

    # then ask session name
    session = input('Session name: ').strip().replace(' ', '_') or 'default'
    cfg = load_config()

    # initialize Piper
    piper = C_PiperInterface_V2(CAN_IFACE)
    piper.ConnectPort()
    while not piper.EnablePiper():
        time.sleep(0.1)
    for i in range(1, NUM_JOINTS+1):
        piper.JointMaxAccConfig(i, PIPER_ACCEL)
        piper.MotorMaxSpdSet(i, PIPER_SPEED)
    
    # serial for gripper
    ser = serial.Serial(RS485_PORT, RS485_BAUD, timeout=0.01)

    # start background threads
    threads = [
        threading.Thread(target=reader_thread, args=(cfg,), daemon=True),
        threading.Thread(target=piper_reader_thread, args=(piper,), daemon=True),
        threading.Thread(target=sender_thread, args=(piper,), daemon=True),
        threading.Thread(target=gripper_thread, args=(ser,), daemon=True)
    ]
    for t in threads:
        t.start()

    # control loop
    term = Terminal()
    try:
        with term.cbreak():
            while True:
                print("Press 's' to start recording, 't' to end session.")
                key = term.inkey(timeout=None).lower()
                if key == 's':
                    stop_flag = {'stop': False}
                    cam_t = threading.Thread(
                        target=camera_thread,
                        args=(state, state_lock, stop_flag, ser, session)
                    )
                    cam_t.start()
                    print('Recording… press q to stop')
                    while term.inkey(timeout=None).lower() != 'q':
                        pass
                    stop_flag['stop'] = True
                    cam_t.join()
                    print('Trial stopped.')
                elif key == 't':
                    print('✱ Ending session.')
                    break
    except KeyboardInterrupt:
        print('\nInterrupted; ending.')

    # shutdown
    event_stop.set()
    for t in threads:
        t.join()
    ser.close()
    pipeline.shutdown()
    z.close()
    print('✅ Goodbye.')

if __name__ == '__main__':
    main()
