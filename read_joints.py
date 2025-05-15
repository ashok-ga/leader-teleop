#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# === Configuration ===
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'
PROTOCOL_VERSION = 2.0

# XL430 control table for joints 0–5
ADDR_PRESENT_POSITION_XL430 = 132
LEN_PRESENT_POSITION_XL430 = 4

# XL320 control table for joint 6
ADDR_PRESENT_POSITION_XL320 = 37
LEN_PRESENT_POSITION_XL320 = 2

# === Initialization ===
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("❌ Failed to open the port")
    exit()
if not portHandler.setBaudRate(BAUDRATE):
    print("❌ Failed to set baudrate")
    exit()

# GroupBulkRead for XL430 joints 0–5
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

for dxl_id in range(0, 6):
    success = groupBulkRead.addParam(dxl_id, ADDR_PRESENT_POSITION_XL430, LEN_PRESENT_POSITION_XL430)
    if not success:
        print(f"[ID:{dxl_id}] ❌ addParam failed")
        exit()

print("✅ Starting read loop. Press Ctrl+C to exit.")

try:
    while True:
        start_time = time.time()

        # Read bulk (IDs 0–5)
        dxl_comm_result = groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"❌ BulkRead failed: {packetHandler.getTxRxResult(dxl_comm_result)}")
            continue

        # Print XL430 motor positions
        for dxl_id in range(0, 6):
            available = groupBulkRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION_XL430, LEN_PRESENT_POSITION_XL430)
            if available:
                raw_pos = groupBulkRead.getData(dxl_id, ADDR_PRESENT_POSITION_XL430, LEN_PRESENT_POSITION_XL430)
                print(f"[ID:{dxl_id}] Present Position (XL430): {raw_pos}")
            else:
                print(f"[ID:{dxl_id}] ⚠️ Data not available")

        # Read XL320 (ID 6) separately
        pos, comm_result, error = packetHandler.read2ByteTxRx(portHandler, 6, ADDR_PRESENT_POSITION_XL320)
        if comm_result == COMM_SUCCESS and error == 0:
            print(f"[ID:6] Present Position (XL320): {pos}")
        else:
            print(f"[ID:6] ❌ Read failed: Comm={comm_result}, Error={error}")

        # FPS
        end_time = time.time()
        loop_time = end_time - start_time
        print(f"FPS: {1.0 / loop_time:.2f}\n")

        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n🛑 Terminating.")

finally:
    portHandler.closePort()
