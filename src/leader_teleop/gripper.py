from logging import debug, error, warning
import struct
import time
import threading
import serial

from leader_teleop.buffer.data_sync_buffer import DataSyncBuffer
from leader_teleop.config.utils import device_config

SERVO_ID = 1
GRIP_SPEED, GRIP_ACCEL = 7500, 0
GD_LOW = 0
GD_HIGH = 25
RS485_BAUD = 115200


class RS485GripperInterface:
    def __init__(
        self,
        diffs,
        servo_cmds,
        servo_pos,
        rs485_device,
        pos_min,
        pos_max,
        poll_frequency=200.0,
    ):
        self.diffs = diffs
        self.servo_cmds = servo_cmds
        self.servo_pos = servo_pos
        self.pos_min = pos_min
        self.pos_max = pos_max

        self.poll_frequency = poll_frequency
        self._stop_event = threading.Event()
        self._thread = None
        self.rs485_device = rs485_device
        self._serial_init(rs485_device)

        time.sleep(0.1)  # Allow serial port to stabilize

    def _serial_init(self, rs485_device):
        self.ser = serial.Serial(
            rs485_device,
            RS485_BAUD,
            timeout=0.01,
        )

    def calculate_checksum(self, packet):
        checksum = sum(packet[2:]) & 0xFF
        return (~checksum) & 0xFF

    def build_move_command(self, servo_id, position, speed, accel):
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
        packet[-1] = self.calculate_checksum(packet)
        return bytes(packet)

    def build_position_query_packet(self, servo_id):
        packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x38, 0x02]
        packet.append(self.calculate_checksum(packet))
        return bytes(packet)

    def make_grip_pkt(self, pos: int) -> bytes:
        hdr = bytearray([0xFF, 0xFF, SERVO_ID, 0x08, 0x03, 0x2A])
        pkt = hdr + struct.pack("<HHB", pos, GRIP_SPEED, GRIP_ACCEL) + b"\x00"
        pkt[-1] = ~sum(pkt[2:]) & 0xFF
        return bytes(pkt)

    def read_servo_position(self, servo_id: int, timeout_s: float = 0.03) -> int | None:
        """
        Returns absolute position 0-4095.
        Returns None on timeout / bad checksum.
        Raises RuntimeError if servo reports an error byte.
        """
        self.ser.reset_input_buffer()
        self.ser.write(self.build_position_query_packet(servo_id))

        deadline = time.time() + timeout_s
        buf = bytearray()

        while time.time() < deadline:
            # ▸ 1. Pull in whatever is waiting (at least one byte)
            buf.extend(self.ser.read(self.ser.in_waiting or 1))

            # print(f"{self.rs485_device} Buffer {buf}")

            # ▸ 2. Scan buf for a complete frame
            while len(buf) >= 4:  # we have header+length
                if buf[0:2] != b"\xff\xff":
                    buf.pop(0)  # resync to header
                    continue

                if buf[2] != servo_id:
                    buf.pop(0)  # not our ID → resync
                    continue

                length = buf[3]
                need = 4 + length
                if len(buf) < need:  # wait for rest
                    break

                frame = buf[:need]
                del buf[:need]  # consume it

                # ▸ 3. Verify checksum
                if (~sum(frame[2:-1]) & 0xFF) != frame[-1]:
                    continue  # bad frame → discard

                err = frame[4]
                if err:
                    warning(f"{self.rs485_device} Servo error: 0x{err:02X}")

                if length == 2:  # status frame → ignore
                    continue

                if length == 4:  # this *is* the reply
                    pos = frame[5] | (frame[6] << 8)
                    return pos

                # any other length → ignore and keep scanning

        return None  # timeout

    def _gripper_loop(self):
        interval = 1.0 / self.poll_frequency
        last_pos = self.pos_min
        while not self._stop_event.is_set():
            buf = self.diffs[-1][1] if len(self.diffs) > 0 else {}

            gd = buf.get("gripper", 0.0)
            debug(f"{self.rs485_device}, Gripper command: {gd}")

            if self.rs485_device == device_config["right_rs485_device"]:
                gd = -gd

            # print(f"Gripper command : {gd}, Gripper Device: {self.rs485_device}")
            # Clamp and map to position
            gd = min(max(gd, GD_LOW), GD_HIGH)

            # if self.rs485_device == device_config["left_rs485_device"]:
            gd = GD_HIGH - gd

            self.servo_cmds.add(gd)

            pos = int(
                self.pos_min
                + ((gd - GD_LOW) / (GD_HIGH - GD_LOW)) * (self.pos_max - self.pos_min)
            )

            # print(
            #     f"Gripper position, gripper: {pos}, Gripper Device: {self.rs485_device}"
            # )

            if pos != last_pos:
                self.ser.write(self.make_grip_pkt(pos))

            actual_pos = self.read_servo_position(SERVO_ID)

            if actual_pos is None:
                actual_pos = last_pos
            else:
                last_pos = actual_pos

            normalized_pos = min(
                max((actual_pos - self.pos_min) / (self.pos_max - self.pos_min), 0.0),
                1.0,
            )
            debug(
                f"{self.rs485_device}, Gripper actual position: {actual_pos}, normalized: {normalized_pos}"
            )
            self.servo_pos.add(normalized_pos)

            time.sleep(interval)

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._gripper_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
        self.ser.close()


class MockRS485GripperInterface(RS485GripperInterface):
    def __init__(
        self, diffs, servo_cmds, servo_pos, rs485_device, poll_frequency=200.0
    ):
        super().__init__(
            diffs,
            servo_cmds,
            servo_pos,
            rs485_device,
            poll_frequency=poll_frequency,
        )

    def _serial_init(self, rs485_device):
        # Mock implementation does not require serial initialization
        pass

    def _gripper_loop(self):
        interval = 1.0 / self.poll_frequency
        while not self._stop_event.is_set():
            self.servo_cmds.add(0.0)
            self.servo_pos.add(0.0)

            time.sleep(interval)


def gripper_init(data_sync_buffer):
    """
    Initialize the RS485GripperInterface with a DataSyncBuffer.
    """
    right_grip = RS485GripperInterface(
        diffs=data_sync_buffer.get_buffer("right_diffs"),
        servo_cmds=data_sync_buffer.get_buffer("right_servo_cmds"),
        servo_pos=data_sync_buffer.get_buffer("right_servo_pos"),
        rs485_device=device_config["right_rs485_device"],
        pos_min=700,
        pos_max=1700,
        poll_frequency=200.0,
    )

    left_grip = RS485GripperInterface(
        diffs=data_sync_buffer.get_buffer("left_diffs"),
        servo_cmds=data_sync_buffer.get_buffer("left_servo_cmds"),
        servo_pos=data_sync_buffer.get_buffer("left_servo_pos"),
        rs485_device=device_config["left_rs485_device"],
        pos_min=1130,
        pos_max=2000,
        poll_frequency=200.0,
    )

    right_grip.start()
    left_grip.start()

    return right_grip, left_grip


def gripper_shutdown(right_grip, left_grip):
    """
    Shutdown the RS485GripperInterface instances.
    """
    right_grip.stop()
    left_grip.stop()
    print("RS485 grippers stopped.")


def dry_run():
    data_sync_buffer = DataSyncBuffer(
        [
            "right_diffs",
            "right_servo_cmds",
            "right_servo_pos",
            "left_diffs",
            "left_servo_cmds",
            "left_servo_pos",
        ]
    )

    right_grip, left_grip = gripper_init(data_sync_buffer)

    data_sync_buffer.get_buffer("right_diffs").add({"gripper": 0.0})
    data_sync_buffer.get_buffer("left_diffs").add({"gripper": 0.0})

    print("Testing Left Gripper...")
    for i in range(1):
        for g in [
            0.0,
            GD_HIGH / 4,
            GD_HIGH / 2,
            GD_HIGH,
            GD_HIGH / 2,
            GD_HIGH / 4,
            0.0,
        ]:
            data_sync_buffer.get_buffer("left_diffs").add({"gripper": g})
            time.sleep(1)

    print("Testing Right Gripper...")
    for i in range(1):
        for g in [
            0,
            GD_HIGH / 4,
            GD_HIGH / 2,
            GD_HIGH,
            GD_HIGH / 2,
            GD_HIGH / 4,
            0,
        ]:
            data_sync_buffer.get_buffer("right_diffs").add({"gripper": -g})
            time.sleep(1)

    print("right_servo_pos:", data_sync_buffer.get_buffer("right_servo_pos").data)
    print("left_servo_pos:", data_sync_buffer.get_buffer("left_servo_pos").data)

    gripper_shutdown(right_grip, left_grip)

    print("Dry run completed.")


if __name__ == "__main__":
    dry_run()
