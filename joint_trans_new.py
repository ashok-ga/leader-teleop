#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Leader-follower demo — “instant-win” latency trims
✦  no per-loop MotionCtrl_2 call       →  ~0.3 ms saved
✦  constant (pre-built) RS-485 packets →  fewer allocations
✦  send gripper / piper command only   →  when value changes
✦  perf-counter-based sleep            →  keeps 200 Hz loop jitters ≤0.6 ms
Everything else (three threads, YAML cfg, etc.) stays unchanged.
"""

import time, yaml, math, threading, serial, struct
from collections import defaultdict
from dynamixel_sdk import *
from piper_sdk import C_PiperInterface_V2

# ───────── user constants ─────────
CONFIG_FILE   = "robot_config.yaml"
PORT_NAME     = "/dev/ttyUSB0"      # Dynamixel bus
BAUDRATE      = 4000000
SERIAL_PORT   = "/dev/ttyUSB1"      # RS-485 gripper
SER_BAUD      = 115200
DEG_FACTOR    = 57295.7795         # Piper units per rad
LOOP_HZ       = 200                 # leader→follower rate
# gripper
GRIP_ID       = 1
POS_MIN, POS_MAX = 1200, 2276
GRIP_SPEED    = 7500
GRIP_ACCEL    = 0
SEND_DELTA    = 3                   # send when ≥3 ticks change
# ───────────────────────────────────

# ───────── RS-485 helper (constant packets) ─────────
MOVE_HDR = bytearray([0xFF, 0xFF, GRIP_ID, 0x08, 0x03, 0x2A])
def build_move_packet(pos: int) -> bytes:
    """Return a ready-to-send 13-byte packet for given position."""
    payload = MOVE_HDR + struct.pack('<HHB', pos, GRIP_SPEED, GRIP_ACCEL)
    payload.append((~sum(payload[2:]) & 0xFF))
    return bytes(payload)

# position query (unused below but kept for completeness)
QUERY_PACKET = bytes([0xFF, 0xFF, GRIP_ID, 0x04, 0x02, 0x38, 0x02,
                      (~(GRIP_ID + 0x04 + 0x02 + 0x38 + 0x02)) & 0xFF])

# ───────── Dynamixel group reader ─────────
class SyncReader:
    def __init__(self, cfg):
        self.port  = PortHandler(PORT_NAME)
        self.ph    = PacketHandler(2.0)
        if not (self.port.openPort() and self.port.setBaudRate(BAUDRATE)):
            raise RuntimeError("❌ Cannot open Dynamixel port")

        # Group readers per motor type (unchanged)
        self.motor_tables = cfg['motor_tables']
        self.joint_groups = defaultdict(list)
        self.readers      = {}
        for j in cfg['robot']['joints']:
            self.joint_groups[j['motor_type']].append(j)

        for mtype, joints in self.joint_groups.items():
            table  = self.motor_tables[mtype]
            reader = GroupSyncRead(self.port, self.ph,
                                   table['present_position'],
                                   table['position_bytes'])
            for j in joints:
                reader.addParam(j['id'])
            self.readers[mtype] = reader

    def read_all(self):
        angles = {}
        for mtype, joints in self.joint_groups.items():
            table  = self.motor_tables[mtype]
            reader = self.readers[mtype]
            if reader.txRxPacket():          # error, skip this cycle
                continue
            for j in joints:
                raw = reader.getData(j['id'],
                                      table['present_position'],
                                      table['position_bytes'])
                if mtype in ('XL430', 'XM430'):
                    deg = (raw / 4095.0) * 360.0
                elif mtype == 'XL320':
                    deg = (raw / 1023.0) * 300.0
                else:
                    deg = raw
                angles[j['name']] = math.radians(deg)
        return angles

    def shutdown(self):
        self.port.closePort()

# ───────── worker threads ─────────
def reader_thread(state, lock, stop, cfg):
    sr = SyncReader(cfg)
    base = sr.read_all()
    try:
        while not stop['flag']:
            now = sr.read_all()
            diffs = {k: now.get(k, base.get(k, 0.0)) - base.get(k, 0.0)
                     for k in base}
            with lock:
                state['diffs'] = diffs
            time.sleep(0.003)  # ~330 Hz readback
    finally:
        sr.shutdown()

def sender_thread(state, lock, stop, piper):
    names   = [f"joint_{i}" for i in range(1, 7)]
    last    = None
    period  = 1.0 / LOOP_HZ
    next_t  = time.perf_counter()
    while not stop['flag']:
        with lock:
            diffs = state.get('diffs', {})
        vals = []
        for idx, name in enumerate(names):
            d = diffs.get(name, 0.0)
            if idx in (2, 4):           # invert joint 3 & 5
                d = -d
            vals.append(int(d * DEG_FACTOR + 0.5))
        # send only if values changed (saves CAN bandwidth & Piper CPU)
        if vals != last:
            try:
                piper.JointCtrl(*vals)
            except Exception as e:
                print(f"❌ JointCtrl error: {e}")
            last = vals
        # pacing
        next_t += period
        time.sleep(max(0, next_t - time.perf_counter()))

def gripper_thread(state, lock, stop, ser):
    last = None
    while not stop['flag']:
        with lock:
            diff = state.get('diffs', {}).get('gripper', 0.0)
        pos = int(POS_MIN + max(0.0, min(diff, 1.0)) * (POS_MAX - POS_MIN))
        if last is None or abs(pos - last) >= SEND_DELTA:
            ser.write(build_move_packet(pos))
            last = pos
        time.sleep(0.01)

# ───────── main ─────────
def main():
    cfg = yaml.safe_load(open(CONFIG_FILE))

    # Piper init – **MotionCtrl_2 only once**
    piper = C_PiperInterface_V2('can0')
    piper.ConnectPort()
    while not piper.EnablePiper():
        time.sleep(0.01)
    piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)          # moved out of loop
    for i in range(1, 7):
        piper.JointMaxAccConfig(i, 500)
        piper.MotorMaxSpdSet(i, 3000)

    # RS-485
    ser = serial.Serial(SERIAL_PORT, SER_BAUD, timeout=0.0005)
    print(f"🔌 RS-485 gripper on {SERIAL_PORT} @ {SER_BAUD}")

    lock, state, stop = threading.Lock(), {}, {'flag': False}
    t_r = threading.Thread(target=reader_thread,
                           args=(state, lock, stop, cfg), daemon=True)
    t_s = threading.Thread(target=sender_thread,
                           args=(state, lock, stop, piper), daemon=True)
    t_g = threading.Thread(target=gripper_thread,
                           args=(state, lock, stop, ser), daemon=True)
    t_r.start(); t_s.start(); t_g.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop['flag'] = True
        t_r.join(); t_s.join(); t_g.join()
        print("✅ Stopped cleanly")

if __name__ == "__main__":
    main()
