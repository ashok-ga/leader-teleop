import struct
import time
import serial
from leader_teleop.config.utils import device_config

SERVO_ID = 1
# gripper extremes
pos_min, pos_max = 730, 1730
GRIP_SPEED, GRIP_ACCEL = 7500, 0
GD_LOW = 0
GD_HIGH = 25
RS485_BAUD = 115200

ser = serial.Serial(device_config["rs485_device"], RS485_BAUD, timeout=0.01)


def calculate_checksum(packet):
    checksum = sum(packet[2:]) & 0xFF
    return (~checksum) & 0xFF


def build_move_command(servo_id, position, speed, accel):
    packet = [
        0xFF,
        0xFF,
        servo_id,
        0x08,
        0x03,
        0x2A,
        position & 0xFF,
        (position >> 8) & 0xFF,
        speed & 0xFF,
        (speed >> 8) & 0xFF,
        accel,
        0x00,
    ]
    packet[-1] = calculate_checksum(packet)
    return bytes(packet)


def build_position_query_packet(servo_id):
    packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x38, 0x02]
    packet.append(calculate_checksum(packet))
    return bytes(packet)


# pre-built RS-485 helper
def make_grip_pkt(pos: int) -> bytes:
    hdr = bytearray([0xFF, 0xFF, SERVO_ID, 0x08, 0x03, 0x2A])
    pkt = hdr + struct.pack("<HHB", pos, GRIP_SPEED, GRIP_ACCEL) + b"\x00"
    pkt[-1] = ~sum(pkt[2:]) & 0xFF
    return bytes(pkt)


def read_servo_position(ser, servo_id):
    query = build_position_query_packet(servo_id)
    ser.reset_input_buffer()
    ser.write(query)
    timeout = time.time() + 0.01
    resp = bytearray()
    while time.time() < timeout:
        if ser.in_waiting:
            resp.extend(ser.read(ser.in_waiting))
    for i in range(len(resp) - 7):
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] == servo_id:
            return resp[i + 5] | (resp[i + 6] << 8)
    return -1


def gripper_thread(event_stop, data_sync_buffer, poll_frequency=200.0):
    """
    Controls the gripper using command values from the DataSyncBuffer and publishes current position back.
    """
    interval = 1.0 / poll_frequency
    last_pos = None

    while not event_stop.is_set():
        # Read latest gripper delta value
        buf = (
            data_sync_buffer.get_buffer("diffs")[-1][1]
            if len(data_sync_buffer.get_buffer("diffs")) > 0
            else {}
        )
        gd = buf.get("gripper", 0.0)
        print("Command Pos", gd)
        data_sync_buffer.get_buffer("ServoCmd").add(gd)

        # Clamp and map to position
        gd = min(max(gd, GD_LOW), GD_HIGH)
        pos = int(pos_min + ((gd - GD_LOW) / (GD_HIGH - GD_LOW)) * (pos_max - pos_min))

        if pos != last_pos:
            ser.write(make_grip_pkt(pos))
            last_pos = pos

        actual_pos = read_servo_position(ser, SERVO_ID)
        print("Actual Servo Position:", actual_pos)
        normalized_pos = min(
            max((actual_pos - pos_min) / (pos_max - pos_min), 0.0), 1.0
        )
        data_sync_buffer.get_buffer("ServoPos").add(normalized_pos)

        time.sleep(interval)
