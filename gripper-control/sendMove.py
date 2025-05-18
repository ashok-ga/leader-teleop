import serial
import time
import random
import threading

# Serial configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
SERVO_ID = 1
POSITION_1 = 700
POSITION_2 = 1910
SERVO_SPEED = 7500
SERVO_ACCEL = 0

current_position = -1
lock = threading.Lock()

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
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    return bytes(packet)

def read_servo_position(ser, servo_id):
    query_packet = build_position_query_packet(servo_id)
    ser.flushInput()
    ser.write(query_packet)

    timeout = time.time() + 0.005  # 5ms timeout
    response = []
    while time.time() < timeout:
        if ser.in_waiting:
            response.extend(ser.read(ser.in_waiting))

    for i in range(len(response) - 7):
        if response[i] == 0xFF and response[i + 1] == 0xFF and response[i + 2] == servo_id:
            position = response[i + 5] | (response[i + 6] << 8)
            return position

    return -1  # Failed to read

def move_servo_task(ser):
    global current_position
    interval = 0.01  # 100 Hz
    next_time = time.time()
    while True:
        position = random.randint(POSITION_1, POSITION_2)
        move_packet = build_move_command(SERVO_ID, position, SERVO_SPEED, SERVO_ACCEL)
        ser.write(move_packet)
        with lock:
            current_position = position
        next_time += interval
        time.sleep(max(0, next_time - time.time()))

def query_servo_task(ser):
    interval = 0.01  # 100 Hz
    next_time = time.time()
    while True:
        feedback_position = read_servo_position(ser, SERVO_ID)
        with lock:
            commanded_position = current_position
        print(f"Commanded Position: {commanded_position}, Feedback Position: {feedback_position}")
        next_time += interval
        time.sleep(max(0, next_time - time.time()))

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001) as ser:
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

        threading.Thread(target=move_servo_task, args=(ser,), daemon=True).start()
        threading.Thread(target=query_servo_task, args=(ser,), daemon=True).start()

        while True:
            time.sleep(1)

if __name__ == "__main__":
    main()
