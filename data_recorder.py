import os
import time
import csv
import threading
from datetime import datetime

import numpy as np
import pyzed.sl as sl
import serial
import yaml
import math
from collections import defaultdict
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead
from piper_sdk import C_PiperInterface_V2
from gstreamer_recorder import GstreamerRecorder

# Paths and ports
CONFIG_FILE = "robot_config.yaml"
DXL_PORT = '/dev/ttyUSB0'
DXL_BAUD = 1000000
CAN_IFACE = 'can0'
RS485_PORT = '/dev/ttyUSB1'
RS485_BAUD = 115200
POSITION_KEY = 'present_position'
SERVO_ID = 1
# Piper control defaults
PIPER_ACCEL = 500
PIPER_SPEED = 3000
# Gripper servo limits
POSITION_1 = 1270
POSITION_2 = 2276
SERVO_SPEED = 7500
SERVO_ACCEL = 0
# Conversion factors
DEG_FACTOR = 57295.7795
NUM_JOINTS = 6

# Global Piper interface
piper = None
home_pos = [0,0,0,0,0,0,0]

def count_total_files(directory):
    total = sum(len(files) for _, _, files in os.walk(directory))
    return total


def calculate_checksum(packet):
    checksum = sum(packet[2:]) & 0xFF
    return (~checksum) & 0xFF


def build_move_command(servo_id, position, speed, accel):
    packet = [0xFF, 0xFF, servo_id, 0x08, 0x03, 0x2A,
              position & 0xFF, (position >> 8) & 0xFF,
              speed & 0xFF, (speed >> 8) & 0xFF,
              accel, 0x00]
    packet[-1] = calculate_checksum(packet)
    return bytes(packet)


def build_position_query_packet(servo_id):
    packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x38, 0x02]
    packet.append(calculate_checksum(packet))
    return bytes(packet)


def read_servo_position(ser, servo_id):
    query = build_position_query_packet(servo_id)
    ser.reset_input_buffer()
    ser.write(query)
    timeout = time.time() + 0.01
    resp = bytearray()
    while time.time() < timeout:
        if ser.in_waiting:
            resp.extend(ser.read(ser.in_waiting))
    for i in range(len(resp)-7):
        if resp[i]==0xFF and resp[i+1]==0xFF and resp[i+2]==servo_id:
            return resp[i+5] | (resp[i+6]<<8)
    return -1

def send_to_origin(state, position):
        joint_0 = round(position[0]*DEG_FACTOR)
        joint_1 = round(position[1]*DEG_FACTOR)
        joint_2 = round(position[2]*DEG_FACTOR)
        joint_3 = round(position[3]*DEG_FACTOR)
        joint_4 = round(position[4]*DEG_FACTOR)
        joint_5 = round(position[5]*DEG_FACTOR)
        piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        state
        


class SyncReader:
    def __init__(self, config):
        self.config = config
        self.port_handler = PortHandler(DXL_PORT)
        self.packet_handler = PacketHandler(2.0)
        if not self.port_handler.openPort():
            raise RuntimeError("❌ Failed to open port")
        if not self.port_handler.setBaudRate(DXL_BAUD):
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



def reader_thread(state, lock, stop, config):
    rd = SyncReader(config)
    init = rd.read_all()
    try:
        while not stop['stop']:
            cur = rd.read_all()
            diffs = {n:cur.get(n,init[n])-init[n] for n in init}
            with lock: state['diffs']=diffs
            time.sleep(0.01)
    finally: rd.shutdown()


def piper_reader_thread(state, lock, stop):
    """Continuously read Piper joint messages and store radians."""
    try:
        while not stop['stop']:
            try:
                msg = piper.GetArmJointMsgs()
                js = msg.joint_state
                raw_vals = [getattr(js, f'joint_{i}', None) for i in range(1, NUM_JOINTS+1)]
                angles_rad = []
                for raw in raw_vals:
                    if raw is None:
                        angles_rad.append(None)
                    else:
                        # raw is Piper units per radian
                        angles_rad.append(raw / DEG_FACTOR)
                with lock:
                    state['piper_angles'] = angles_rad
            except Exception:
                pass
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


def sender_thread(state, lock, stop):
    names=[f'joint_{i}' for i in range(1,NUM_JOINTS+1)]
    while not stop['stop']:
        with lock: diffs=state.get('diffs',{})
        try: piper.MotionCtrl_2(1,1,100,0)
        except: pass
        cmds=[]
        for i,n in enumerate(names):
            d=diffs.get(n,0);
            if i in(2,4): d=-d
            cmds.append(int(d*DEG_FACTOR+0.5))
        try: piper.JointCtrl(*cmds)
        except: pass
        time.sleep(0.005)


def gripper_thread(state, lock, stop, ser):
    while not stop['stop']:
        with lock: gd=state.get('diffs',{}).get('gripper',0)
        pos=int(POSITION_1+min(max(gd,0),1)*(POSITION_2-POSITION_1))
        ser.write(build_move_command(SERVO_ID,pos,SERVO_SPEED,SERVO_ACCEL))
        time.sleep(0.01)


def camera_thread(state, lock, stop, ser, name):
    global piper
    z=sl.Camera(); ip=sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,camera_fps=30)
    if z.open(ip)!=sl.ERROR_CODE.SUCCESS: return
    rt=sl.RuntimeParameters();L,R=sl.Mat(),sl.Mat()
    info=z.get_camera_information();w,h=info.camera_configuration.resolution.width,info.camera_configuration.resolution.height
    ts=datetime.now().strftime('%Y%m%d_%H%M%S');od=f'recordings/{name}_{ts}';os.makedirs(od,exist_ok=True)
    cf=open(f'{od}/data.csv','w',newline='');wr=csv.writer(cf)
    hdr=['Timestamp','ServoCmd','ServoPos']+[f'joint_{i}' for i in range(1,NUM_JOINTS+1)]
    wr.writerow(hdr)
    rec=GstreamerRecorder(f'{od}/video.mp4');fc,st=0,time.time()
    try:
        while not stop['stop']:
            if z.grab(rt)==sl.ERROR_CODE.SUCCESS:
                z.retrieve_image(L,sl.VIEW.LEFT);z.retrieve_image(R,sl.VIEW.RIGHT)
                lf,rf=L.get_data(),R.get_data();stereo=np.zeros((h,w*2,4),np.uint8)
                stereo[:,:w,:]=lf;stereo[:,w:,:]=rf
                t=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                with lock: gd=state.get('diffs',{}).get('gripper',0);pa=state.get('piper_angles',[None]*NUM_JOINTS)
                sr=read_servo_position(ser,SERVO_ID)
                sn=min(max((sr-POSITION_1)/(POSITION_2-POSITION_1),0),1)
                wr.writerow([t,gd,sn]+pa)
                rec.push_frame(stereo.tobytes());fc+=1
    finally:
        rec.close();cf.close();z.close()
        print(f'Total files: {count_total_files("recordings")}')


def load_config(p=CONFIG_FILE):
    return yaml.safe_load(open(p))


def main():
    global piper

    # 1) Session setup
    session_name = input('Enter session name: ').strip().replace(' ', '_')
    cfg = load_config()

    # 2) Piper init
    piper = C_PiperInterface_V2(CAN_IFACE)
    piper.ConnectPort()
    while not piper.EnablePiper():
        time.sleep(0.01)
    for i in range(1, NUM_JOINTS+1):
        piper.JointMaxAccConfig(i, PIPER_ACCEL)
        piper.MotorMaxSpdSet(i, PIPER_SPEED)
        time.sleep(0.02)

    # 3) Serial for gripper
    ser = serial.Serial(RS485_PORT, RS485_BAUD, timeout=0.01)

    # 4) Shared state + stop flags
    lock = threading.Lock()
    state = {}
    stop_all = {'stop': False}

    # 5) Start always-running threads
    background_threads = [
        threading.Thread(
            target=reader_thread, args=(state, lock, stop_all, cfg), daemon=True
        ),
        threading.Thread(
            target=piper_reader_thread, args=(state, lock, stop_all), daemon=True
        ),
        threading.Thread(
            target=sender_thread, args=(state, lock, stop_all), daemon=True
        ),
        threading.Thread(
            target=gripper_thread, args=(state, lock, stop_all, ser), daemon=True
        ),
    ]
    for t in background_threads:
        t.start()

    # 6) Trial control loop
    try:
        while True:
            cmd = input("\nPress 's' to **start** recording, 't' to **end** session: ").strip().lower()
            if cmd == 's':
                # Launch one camera trial
                trial_stop = {'stop': False}
                cam_thread = threading.Thread(
                    target=camera_thread,
                    args=(state, lock, trial_stop, ser, session_name),
                    daemon=False
                )
                cam_thread.start()
                print("→ Recording...  (when you want to stop this trial, press 'q' + Enter)")

                # Wait for 'q' to end the trial
                while True:
                    subcmd = input().strip().lower()
                    if subcmd == 'q':
                        trial_stop['stop'] = True
                        cam_thread.join()
                        print("← Trial stopped and files closed.")
                        break
                    else:
                        print("  (press 'q' + Enter to stop this recording)")

            elif cmd == 't':
                print("✱ Ending session.")
                break

            else:
                print("Unrecognized command. Please press 's' or 't'.")

    except KeyboardInterrupt:
        print("\nInterrupted by user; ending session.")

    # 7) Tear down background threads
    stop_all['stop'] = True
    for t in background_threads:
        t.join()
    ser.close()
    print("All threads stopped. Goodbye.")


if __name__ == '__main__':
    main()