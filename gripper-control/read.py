import serial
import time

# Serial configuration
SERIAL_PORT = '/dev/ttyCH341USB0'
BAUD_RATE = 115200
SERVO_ID = 1

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

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) as ser:
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

        while True:
            servo_position = read_servo_position(ser, SERVO_ID)

            if servo_position != -1:
                print(f"Current Servo Position: {servo_position}")
            else:
                print("Failed to read servo position.")

            time.sleep(0.1)

if __name__ == "__main__":
    main()
