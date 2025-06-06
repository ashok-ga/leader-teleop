from logging import debug
import math
import threading
import time
from typing import Optional

from piper_sdk import C_PiperInterface_V2

from leader_teleop.buffer.data_sync_buffer import Buffer

PIPER_ACCEL = 500
PIPER_SPEED = 3000
DEG_FACTOR = 57295.7795
NUM_JOINTS = 6
LOOP_HZ = 200


# Joint limits in degrees (consistent with the output format)
JOINT_LIMITS_RAD = [
    (-2.6179, 2.6179),  # joint1 [-150°, 150°]
    (0.0, 3.14),  # joint2 [0°, 180°]
    (-2.967, 0.0),  # joint3 [-170°, 0°]
    (-1.745, 1.745),  # joint4 [-100°, 100°]
    (-1.22, 1.22),  # joint5 [-70°, 70°]
    (-2.09439, 2.09439),  # joint6 [-120°, 120°]
]


class PiperInterface:
    def __init__(
        self,
        diffs: Buffer,
        eef_pose: Buffer,
        can: str,
        accel: int = PIPER_ACCEL,
        speed: int = PIPER_SPEED,
        loop_hz: float = LOOP_HZ,
    ):
        self.can = can
        self.accel = accel
        self.speed = speed
        self.loop_hz = loop_hz
        self.diffs = diffs
        self.eef_pose = eef_pose
        self.piper = C_PiperInterface_V2(can)
        self.event_stop = threading.Event()
        self._reader_thread = None
        self._sender_thread = None
        self.connect()

    def connect(self):
        self.piper.ConnectPort()
        self.piper.EnableArm()

        # while not self.piper.EnablePiper():
        #     time.sleep(0.005)
        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        time.sleep(0.1)  # Allow some time for the arm to enable

        debug(
            self.can,
            "Crash Protection config",
            self.piper.GetCrashProtectionLevelFeedback(),
        )
        for i in range(1, NUM_JOINTS + 1):
            self.piper.JointMaxAccConfig(i, self.accel)
            self.piper.MotorMaxSpdSet(i, self.speed)

        time.sleep(0.1)

    def _piper_angle_to_rad(self, angle):
        return angle * math.pi / 180.0 / 1e3

    def _reader(self):
        """Read end-effector pose and write to DataSyncBuffer."""
        interval = 1.0 / self.loop_hz
        i = 0
        while not self.event_stop.is_set():
            if i % (self.loop_hz * 5) == 0:
                debug(self.can, self.piper.GetArmStatus())
                debug(self.can, self.piper.GetArmLowSpdInfoMsgs())
                debug(self.can, self.piper.GetArmHighSpdInfoMsgs())

            msg = self.piper.GetArmEndPoseMsgs()
            js = msg.end_pose
            eef_pose = {
                "x": js.X_axis / 1e6,
                "y": js.Y_axis / 1e6,
                "z": js.Z_axis / 1e6,
                "qx": self._piper_angle_to_rad(js.RX_axis),
                "qy": self._piper_angle_to_rad(js.RY_axis),
                "qz": self._piper_angle_to_rad(js.RZ_axis),
            }
            self.eef_pose.add(eef_pose)
            time.sleep(interval)
            i += 1

    def _sender(self):
        """Send joint position commands based on joint diffs."""
        names = [f"joint_{i}" for i in range(1, NUM_JOINTS + 1)]
        last_cmds = None
        period = 1.0 / self.loop_hz
        next_t = time.perf_counter()

        while not self.event_stop.is_set():
            cmds = []
            buf = self.diffs
            diffs = buf[-1][1] if len(buf) > 0 else {}

            for idx, name in enumerate(names):
                d = -diffs.get(name, 0.0) if idx in (2, 4) else diffs.get(name, 0.0)
                # Clamp d in radians to joint limits
                min_rad, max_rad = JOINT_LIMITS_RAD[idx]
                d = max(min(d, max_rad), min_rad)
                # Convert to int degrees for JointCtrl, as in your code
                d_deg = d * DEG_FACTOR
                cmds.append(int(d_deg))

            if cmds != last_cmds:
                try:
                    # print(self.can, cmds)

                    self.piper.JointCtrl(*cmds)
                    last_cmds = cmds
                except Exception as e:
                    print(f"Error sending joint commands: {e}, for piper {self.can}")
                    last_cmds = None

            next_t += period
            self.event_stop.wait(max(0, next_t - time.perf_counter()))

    def start(self):
        """Start Piper interface threads."""
        if self.diffs is None or self.eef_pose is None:
            raise RuntimeError(
                "data_sync_buffer must be set before starting PiperInterface"
            )
        self.connect()
        self.event_stop.clear()
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._sender_thread = threading.Thread(target=self._sender, daemon=True)
        self._reader_thread.start()
        self._sender_thread.start()

    def stop(self):
        """Stop Piper interface threads."""
        self.event_stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join()
        if self._sender_thread is not None:
            self._sender_thread.join()


def piper_init(data_sync_buffer):
    """
    Initialize the PiperInterface with a DataSyncBuffer.
    """
    piper_right = PiperInterface(
        eef_pose=data_sync_buffer.get_buffer("right_eef_pose"),
        diffs=data_sync_buffer.get_buffer("right_diffs"),
        can="can_right",
    )
    piper_left = PiperInterface(
        eef_pose=data_sync_buffer.get_buffer("left_eef_pose"),
        diffs=data_sync_buffer.get_buffer("left_diffs"),
        can="can_left",
    )
    piper_right.start()
    piper_left.start()

    print("Piper interfaces initialized.")
    return piper_right, piper_left


def piper_shutdown(piper_right, piper_left):
    """
    Shutdown the PiperInterface instances.
    """
    piper_right.stop()
    piper_left.stop()
    print("Piper interfaces stopped.")


def dry_run():
    from leader_teleop.buffer.data_sync_buffer import DataSyncBuffer

    data_sync_buffer = DataSyncBuffer(
        ["right_eef_pose", "right_diffs", "left_eef_pose", "left_diffs"]
    )
    piper_right = PiperInterface(
        eef_pose=data_sync_buffer.get_buffer("right_eef_pose"),
        diffs=data_sync_buffer.get_buffer("right_diffs"),
        can="can_right",
    )
    piper_left = PiperInterface(
        eef_pose=data_sync_buffer.get_buffer("left_eef_pose"),
        diffs=data_sync_buffer.get_buffer("left_diffs"),
        can="can_left",
    )
    piper_right.start()
    piper_left.start()

    # Example: issue some sample commands

    data_sync_buffer.get_buffer("left_diffs").add(
        {f"joint_{i}": 0.0 for i in range(1, NUM_JOINTS + 1)}
    )
    time.sleep(2)

    data_sync_buffer.get_buffer("left_diffs").add(
        {f"joint_{i}": 0.8 if i in (2, 3) else 0.0 for i in range(1, NUM_JOINTS + 1)}
    )
    time.sleep(2)

    data_sync_buffer.get_buffer("left_diffs").add(
        {f"joint_{i}": 0.0 for i in range(1, NUM_JOINTS + 1)}
    )
    time.sleep(2)

    data_sync_buffer.get_buffer("right_diffs").add(
        {f"joint_{i}": 0.0 for i in range(1, NUM_JOINTS + 1)}
    )

    time.sleep(2)
    data_sync_buffer.get_buffer("right_diffs").add(
        {f"joint_{i}": 0.8 if i in (2, 3) else 0.0 for i in range(1, NUM_JOINTS + 1)}
    )

    time.sleep(2)
    data_sync_buffer.get_buffer("right_diffs").add(
        {f"joint_{i}": 0.0 for i in range(1, NUM_JOINTS + 1)}
    )

    time.sleep(2)

    piper_right.stop()
    piper_left.stop()
    print("Dry run completed.")


if __name__ == "__main__":
    dry_run()
