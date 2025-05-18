#!/usr/bin/env python3
import serial
import time

# Serial configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200          # Adjust as per your device's specification
TIMEOUT = 1               # seconds


def decode_message(data_bytes):
    """
    Implement your decoding logic here.
    For demonstration, it prints hexadecimal representation.
    """
    decoded_message = ' '.join(f'{byte:02X}' for byte in data_bytes)
    return decoded_message


def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud rate...")
            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    decoded = decode_message(data)
                    print(f"Received: {decoded}")
                time.sleep(0.01)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nExiting.")


if __name__ == "__main__":
    main()

