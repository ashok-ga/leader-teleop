#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import threading
import struct
import time
import yaml
import math
from datetime import datetime
from collections import defaultdict

import numpy as np
import pyzed.sl as sl
import serial
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead
from piper_sdk import C_PiperInterface_V2 # Assuming this is the correct import
from gsn import GstreamerRecorder # Assuming this is a custom/valid import
from blessed import Terminal

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
LOOP_HZ       = 1000
pos_min, pos_max = 2750, 3898
GRIP_SPEED, GRIP_ACCEL = 7500, 0
READER_MOVING_AVG_WINDOW_SIZE = 8

def make_grip_pkt(pos: int) -> bytes:
    hdr = bytearray([0xFF, 0xFF, SERVO_ID, 0x08, 0x03, 0x2A])
    pkt = hdr + struct.pack('<HHB', pos, GRIP_SPEED, GRIP_ACCEL) + b"\x00"
    pkt[-1] = (~sum(pkt[2:]) & 0xFF)
    return bytes(pkt)

class SyncReader:
    def __init__(self, config):
        self.port = PortHandler(DXL_PORT)
        self.handler = PacketHandler(2.0)
        if not self.port.openPort():
            raise IOError(f"Failed to open Dynamixel port {DXL_PORT}")
        if not self.port.setBaudRate(DXL_BAUD):
            raise IOError(f"Failed to set baud rate {DXL_BAUD} for Dynamixel port")
        
        tables = config['motor_tables']
        groups = defaultdict(list)
        for j in config['robot']['joints']:
            if j['id'] <= NUM_JOINTS + 1:
                groups[j['motor_type']].append(j)
        self.readers = {}
        for mtype, joints in groups.items():
            tbl = tables[mtype]
            addr = tbl[POSITION_KEY]
            length = tbl['position_bytes']
            reader = GroupSyncRead(self.port, self.handler, addr, length)
            for j_info in joints:
                reader.addParam(j_info['id'])
            self.readers[mtype] = {
                'reader': reader,
                'address': addr,
                'length': length,
                'joints': joints
            }

    def read_all(self):
        out = {}
        for info in self.readers.values():
            reader = info['reader']
            comm_result = reader.txRxPacket()
            if comm_result != 0: 
                continue
            
            addr   = info['address']
            length = info['length']
            for j in info['joints']:
                if not reader.isAvailable(j['id'], addr, length):
                    continue
                
                raw = reader.getData(j['id'], addr, length)
                if raw is None: 
                    continue

                if j['name'] == 'gripper':
                    denominator = pos_max - pos_min
                    if denominator == 0:
                        val = 0.0
                    else:
                        val = (raw - pos_min) / denominator
                    val = max(0.0, min(1.0, val))
                else:
                    if j['motor_type'] in ('XL430', 'XM430'):
                        val = math.radians((raw / 4095.0) * 360.0)
                    elif j['motor_type'] == 'XL320':
                        val = math.radians((raw / 1023.0) * 300.0)
                    else:
                        val = raw
                out[j['name']] = val
        return out

    def shutdown(self):
        self.port.closePort()

event_stop = threading.Event()
state: dict = {}
state_lock = threading.Lock()

def reader_thread(cfg):
    rd = SyncReader(cfg)
    history = defaultdict(list) 
    current_sums = defaultdict(float)
    smoothed_base_positions = {} 
    current_smoothed_positions = {}

    initial_joint_names = set()
    for _ in range(5): 
        if event_stop.is_set():
            rd.shutdown()
            return
        raw_vals = rd.read_all()
        if raw_vals:
            initial_joint_names.update(raw_vals.keys())
        event_stop.wait(0.001) 

    if not initial_joint_names:
        rd.shutdown()
        return

    successful_priming_cycles = 0
    priming_attempts = 0
    max_priming_attempts = READER_MOVING_AVG_WINDOW_SIZE * 5 

    while successful_priming_cycles < READER_MOVING_AVG_WINDOW_SIZE and not event_stop.is_set():
        priming_attempts += 1
        if priming_attempts > max_priming_attempts and not any(len(h) == READER_MOVING_AVG_WINDOW_SIZE for h in history.values()):
            rd.shutdown()
            return

        raw_positions = rd.read_all()
        if raw_positions:
            processed_this_cycle = False
            for joint_name, pos in raw_positions.items():
                history[joint_name].append(pos)
                current_sums[joint_name] += pos
                processed_this_cycle = True
                
                if len(history[joint_name]) > READER_MOVING_AVG_WINDOW_SIZE:
                    removed_val = history[joint_name].pop(0)
                    current_sums[joint_name] -= removed_val
            
            if processed_this_cycle:
                all_initial_joints_primed_this_cycle = True
                temp_min_len_this_cycle = READER_MOVING_AVG_WINDOW_SIZE
                for name in initial_joint_names:
                    if len(history[name]) < READER_MOVING_AVG_WINDOW_SIZE:
                         if name not in raw_positions and len(history[name]) < successful_priming_cycles : # if it missed an update and is behind
                              all_initial_joints_primed_this_cycle = False
                    temp_min_len_this_cycle = min(temp_min_len_this_cycle, len(history[name]))

                if all_initial_joints_primed_this_cycle :
                    successful_priming_cycles = temp_min_len_this_cycle

        event_stop.wait(0.0005) 

    if event_stop.is_set(): 
        rd.shutdown()
        return

    for joint_name in initial_joint_names:
        if len(history[joint_name]) == READER_MOVING_AVG_WINDOW_SIZE:
            smoothed_base_positions[joint_name] = current_sums[joint_name] / READER_MOVING_AVG_WINDOW_SIZE
    
    if not smoothed_base_positions:
        rd.shutdown()
        return
    
    with state_lock:
        state['smoothed_base_positions'] = smoothed_base_positions.copy()


    while not event_stop.is_set():
        current_raw_positions = rd.read_all()
        
        if not current_raw_positions:
            event_stop.wait(0.0005)
            continue

        newly_smoothed_positions = {}
        for joint_name, pos in current_raw_positions.items():
            history[joint_name].append(pos) 
            current_sums[joint_name] += pos
            
            if len(history[joint_name]) > READER_MOVING_AVG_WINDOW_SIZE:
                removed_val = history[joint_name].pop(0)
                current_sums[joint_name] -= removed_val
            
            if len(history[joint_name]) == READER_MOVING_AVG_WINDOW_SIZE:
                newly_smoothed_positions[joint_name] = current_sums[joint_name] / READER_MOVING_AVG_WINDOW_SIZE
        
        current_smoothed_positions.update(newly_smoothed_positions)

        diffs = {}
        for joint_name_in_base, base_pos in smoothed_base_positions.items():
            current_smooth_pos = current_smoothed_positions.get(joint_name_in_base)
            if current_smooth_pos is not None:
                diffs[joint_name_in_base] = current_smooth_pos - base_pos
            else:
                diffs[joint_name_in_base] = 0.0 

        with state_lock:
            state['diffs'] = diffs
            state['current_smoothed_positions'] = current_smoothed_positions.copy()
            
        event_stop.wait(0.0005)

    rd.shutdown()

def piper_reader_thread(piper_instance):
    while not event_stop.is_set():
        js_msg = piper_instance.GetArmJointMsgs()
        if js_msg:
            js = js_msg.joint_state
            angles = [getattr(js, f'joint_{i}', 0)/DEG_FACTOR for i in range(1, NUM_JOINTS+1)]
            with state_lock:
                state['piper_angles'] = angles
        event_stop.wait(0.0005)

def sender_thread(piper_instance):
    period = 1.0 / LOOP_HZ
    next_t = time.perf_counter()
    piper_joint_names = [f'joint_{i}' for i in range(1, NUM_JOINTS+1)]
    
    while not event_stop.is_set():
        diffs_from_state = {}
        last_cmds_piper = []
        with state_lock:
            diffs_from_state = state.get('diffs', {})
            last_cmds_piper = state.get('last_cmds')
        
        current_piper_commands = []
        all_diffs_available = True
        if not diffs_from_state : all_diffs_available = False

        if all_diffs_available:
            for idx, name_key in enumerate(piper_joint_names):
                if name_key not in diffs_from_state: # Check if specific joint diff is available
                    all_diffs_available = False
                    break
                raw_diff = diffs_from_state.get(name_key, 0.0) # Default to 0.0 if somehow still missing
                
                d_val = -raw_diff if idx in (2, 4) else raw_diff # Indices 2 and 4 are 3rd and 5th joints (0-indexed)
                current_piper_commands.append(int(d_val * DEG_FACTOR + 0.5))

        if all_diffs_available and current_piper_commands and current_piper_commands != last_cmds_piper:
            piper_instance.JointCtrl(*current_piper_commands)
            with state_lock:
                state['last_cmds'] = current_piper_commands
        
        next_t += period
        wait_time = max(0, next_t - time.perf_counter())
        event_stop.wait(wait_time)

def gripper_thread(ser_instance):
    last_sent_servo_pos = -1 
    while not event_stop.is_set():
        gripper_target_fraction = 0.0
        with state_lock:
            current_smoothed = state.get('current_smoothed_positions', {})
            gripper_target_fraction = current_smoothed.get('gripper', 0.0)
            
        clamped_fraction = max(0.0, min(1.0, gripper_target_fraction))
        target_servo_val = int(pos_min + clamped_fraction * (pos_max - pos_min))
        
        if abs(target_servo_val - last_sent_servo_pos) > 0: # Only send if changed
            try:
                ser_instance.write(make_grip_pkt(target_servo_val))
                last_sent_servo_pos = target_servo_val
                with state_lock: # Store for logging/debug if needed
                    state['last_grip_commanded_fraction'] = clamped_fraction
                    state['last_grip_commanded_servo_val'] = target_servo_val
            except serial.SerialException:
                pass # Handle write error if necessary, e.g. log it
        
        event_stop.wait(0.01) 

def read_servo_position(ser_instance, sid_val):
    pkt  = bytes([0xFF, 0xFF, sid_val, 0x04, 0x02, 0x38, 0x02])
    chk  = (~sum(pkt[2:]) & 0xFF)
    try:
        if not ser_instance.is_open: return 0
        ser_instance.reset_input_buffer()
        ser_instance.write(pkt + bytes([chk]))
    except serial.SerialException:
        return 0 

    deadline = time.time() + 0.02 
    resp = bytearray()
    
    while time.time() < deadline:
        try:
            if not ser_instance.is_open: return 0
            if ser_instance.in_waiting > 0:
                resp.extend(ser_instance.read(ser_instance.in_waiting))
                if len(resp) >= 8: 
                    break 
        except serial.SerialException:
             return 0
        time.sleep(0.001)

    for i in range(len(resp) - 7): 
        if resp[i:i+2] == b"\xFF\xFF" and resp[i+2] == sid_val:
            if resp[i+4] == 0x00: 
                 return resp[i+5] | (resp[i+6] << 8)
    return 0

z_global: sl.Camera = None
pipeline_global: GstreamerRecorder = None
piper_global: C_PiperInterface_V2 = None

def camera_thread(current_state_ref, lock_ref, stop_dict_ref, rs485_ser_ref, session_name_str):
    global z_global, pipeline_global

    rt_params = sl.RuntimeParameters()
    l_image_mat, r_image_mat = sl.Mat(), sl.Mat()
    cam_information = z_global.get_camera_information()
    w_res, h_res = cam_information.camera_configuration.resolution.width, cam_information.camera_configuration.resolution.height
    stereo_array_np = np.ascontiguousarray(np.empty((h_res, w_res * 2, 4), dtype=np.uint8))
    
    ts_folder_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_path_str = f'recordings/{session_name_str}_{ts_folder_str}'
    os.makedirs(output_path_str, exist_ok=True)
    
    csv_file_path_str = f'{output_path_str}/data.csv'
    csv_file = open(csv_file_path_str, 'w', newline='')
    csv_writer_obj = csv.writer(csv_file)
    
    csv_header_list = ['Timestamp', 'ServoCmdFraction', 'ServoPosFraction'] + [f'piper_joint_angle_{i}' for i in range(1, NUM_JOINTS + 1)]
    csv_writer_obj.writerow(csv_header_list)
    
    video_file_path_str = f'{output_path_str}/video.mp4'
    pipeline_global.start_recording(video_file_path_str)
    
    try:
        while not stop_dict_ref['stop']:
            if z_global.grab(rt_params) == sl.ERROR_CODE.SUCCESS:
                z_global.retrieve_image(l_image_mat, sl.VIEW.LEFT)
                z_global.retrieve_image(r_image_mat, sl.VIEW.RIGHT)
                
                stereo_array_np[:, :w_res] = l_image_mat.get_data()
                stereo_array_np[:, w_res:] = r_image_mat.get_data()
                
                current_timestamp_val = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                
                piper_angles_list = [0.0] * NUM_JOINTS
                gripper_cmd_fraction_val = 0.0

                with lock_ref:
                    piper_angles_list = list(current_state_ref.get('piper_angles', [0.0] * NUM_JOINTS))
                    gripper_cmd_fraction_val = current_state_ref.get('last_grip_commanded_fraction', 0.0)

                raw_servo_pos_val = read_servo_position(rs485_ser_ref, SERVO_ID)
                servo_pos_denominator = (pos_max - pos_min)
                normalized_servo_pos_val = 0.0
                if servo_pos_denominator != 0:
                    normalized_servo_pos_val = (raw_servo_pos_val - pos_min) / servo_pos_denominator
                normalized_servo_pos_val = max(0.0, min(1.0, normalized_servo_pos_val))
                
                csv_writer_obj.writerow([current_timestamp_val, gripper_cmd_fraction_val, normalized_servo_pos_val] + piper_angles_list)
                pipeline_global.push_frame(stereo_array_np)
            else:
                time.sleep(0.001)
                
    finally:
        pipeline_global.stop_recording()
        csv_file.close()

def load_config_from_file():
    if not os.path.exists(CONFIG_FILE):
        raise FileNotFoundError(f"Config file '{CONFIG_FILE}' not found.")
    with open(CONFIG_FILE, 'r') as f_cfg:
        return yaml.safe_load(f_cfg)

def main():
    global z_global, pipeline_global, piper_global

    z_global = sl.Camera()
    zed_init_params = sl.InitParameters(
        camera_resolution=sl.RESOLUTION.HD720,
        camera_fps=30,
        sdk_verbose=False
    )
    zed_status = z_global.open(zed_init_params)
    if zed_status != sl.ERROR_CODE.SUCCESS:
        return
    
    pipeline_global = GstreamerRecorder()

    session_name_input = input('Session name: ').strip().replace(' ', '_') or 'default_session'
    
    try:
        application_config = load_config_from_file()
    except FileNotFoundError:
        if z_global and z_global.is_opened():
            z_global.close()
        return

    rs485_serial_port = None

    piper_global = C_PiperInterface_V2(CAN_IFACE) 

    piper_global.ConnectPort()
    while not piper_global.EnablePiper():
        time.sleep(0.005)
    for i in range(1, NUM_JOINTS+1):
        piper_global.JointMaxAccConfig(i, PIPER_ACCEL)
        piper_global.MotorMaxSpdSet(i, PIPER_SPEED)
    
    rs485_serial_port = serial.Serial(RS485_PORT, RS485_BAUD, timeout=0.01)

    background_threads_list = [
        threading.Thread(target=reader_thread, args=(application_config,), daemon=True, name="DXLReader"),
        threading.Thread(target=piper_reader_thread, args=(piper_global,), daemon=True, name="PiperReader"),
        threading.Thread(target=sender_thread, args=(piper_global,), daemon=True, name="PiperSender"),
        threading.Thread(target=gripper_thread, args=(rs485_serial_port,), daemon=True, name="GripperControl")
    ]
    for t_item in background_threads_list:
        t_item.start()

    terminal_interface = Terminal()
    try:
        with terminal_interface.cbreak():
            while True:
                print("Press 's' to start recording, 't' to end session.")
                key_input = terminal_interface.inkey(timeout=None).lower()
                if key_input == 's':
                    camera_stop_event_dict = {'stop': False}
                    camera_control_thread_obj = threading.Thread(
                        target=camera_thread,
                        args=(state, state_lock, camera_stop_event_dict, rs485_serial_port, session_name_input),
                        name="CamRec"
                    )
                    camera_control_thread_obj.start()
                    print('Recording… press q to stop')
                    while terminal_interface.inkey(timeout=None).lower() != 'q':
                        pass # Wait for 'q' to stop recording
                    camera_stop_event_dict['stop'] = True
                    camera_control_thread_obj.join()
                    print('Trial stopped.')
                elif key_input == 't':
                    break # Exit main loop to shutdown
    except KeyboardInterrupt:
        pass # Handle Ctrl+C gracefully
    finally:
        event_stop.set() # Signal all daemon threads to stop
        
        for t_item in background_threads_list:
            if t_item.is_alive():
                t_item.join(timeout=1.5) # Wait a bit longer for threads
        
        if rs485_serial_port and rs485_serial_port.is_open:
            rs485_serial_port.close()

        if pipeline_global: # Assuming pipeline_global might be None if init failed earlier
             pipeline_global.shutdown()
        if z_global and z_global.is_opened():
            z_global.close()

if __name__ == '__main__':
    main()