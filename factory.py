from dynamixel_sdk import *

PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DXL_ID = 3  # Replace with your motor ID
TORQUE_ENABLE_ADDR = 64

def factory_reset(dxl_id):
    portHandler = PortHandler(PORT_NAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        raise RuntimeError("❌ Failed to open port")
    if not portHandler.setBaudRate(BAUDRATE):
        raise RuntimeError("❌ Failed to set baudrate")

    print(f"⚠️ Disabling torque for ID {dxl_id}...")
    packetHandler.write1ByteTxRx(portHandler, dxl_id, TORQUE_ENABLE_ADDR, 0)

    print(f"⚠️ Sending factory reset command to ID {dxl_id}...")
    result, dxl_error = packetHandler.factoryReset(portHandler, dxl_id, 0)
    
    if result == COMM_SUCCESS:
        print("✅ Factory reset command sent successfully")
    else:
        print(f"❌ Communication error: {packetHandler.getTxRxResult(result)}")

    if dxl_error != 0:
        print(f"⚠️ Error code returned: {packetHandler.getRxPacketError(dxl_error)}")

    portHandler.closePort()

if __name__ == "__main__":
    factory_reset(DXL_ID)
