import math
import threading
import time

from piper_sdk import C_PiperInterface_V2

PIPER_ACCEL = 500
PIPER_SPEED = 3000
DEG_FACTOR = 57295.7795
NUM_JOINTS = 6
LOOP_HZ = 200
CAN_IFACE = "can0"


piper = C_PiperInterface_V2(CAN_IFACE)
piper.ConnectPort()

piper.EnableArm()

# while not piper.EnablePiper():
#     time.sleep(0.005)
for i in range(1, NUM_JOINTS + 1):
    piper.JointMaxAccConfig(i, PIPER_ACCEL)
    piper.MotorMaxSpdSet(i, PIPER_SPEED)


def piper_reader_thread(event_stop, data_sync_buffer, poll_frequency=200.0):
    """Reads end-effector pose from Piper and writes to DataSyncBuffer."""
    interval = 1.0 / poll_frequency

    def _piper_angle_to_rad(angle):
        return angle * math.pi / 180.0 / 1e3

    while not event_stop.is_set():
        msg = piper.GetArmEndPoseMsgs()
        js = msg.end_pose

        # Decompose into separate keys (or keep a single dict if preferred)
        eef_pose = {
            "x": js.X_axis / 1e6,
            "y": js.Y_axis / 1e6,
            "z": js.Z_axis / 1e6,
            "qx": _piper_angle_to_rad(js.RX_axis),
            "qy": _piper_angle_to_rad(js.RY_axis),
            "qz": _piper_angle_to_rad(js.RZ_axis),
        }

        data_sync_buffer.get_buffer("eef_pose").add(eef_pose)

        time.sleep(interval)


def piper_sender_thread(event_stop, data_sync_buffer, loop_hz=200.0):
    """Sends joint position commands to Piper based on joint diffs."""
    names = [f"joint_{i}" for i in range(1, NUM_JOINTS + 1)]
    last_cmds = None
    period = 1.0 / loop_hz
    next_t = time.perf_counter()

    while not event_stop.is_set():
        cmds = []

        buf = data_sync_buffer.get_buffer("diffs")
        diffs = buf[-1][1] if len(buf) > 0 else {}

        # print(f"Joint diffs: {diffs}")
        for idx, name in enumerate(names):
            d = -diffs.get(name, 0.0) if idx in (2, 4) else diffs.get(name, 0.0)
            if idx == 1:
                d = max(d, 0.0)
            elif idx == 2:
                d = min(d, 0.0)
            # if idx in (2, 4):
            #     d = min(d, 0.0)
            # elif idx in (1, 3):
            #     d = max(d, 0.0)

            cmds.append(int(d * DEG_FACTOR + 0.5))

        if cmds != last_cmds:
            piper.JointCtrl(*cmds)
            last_cmds = cmds

        next_t += period
        event_stop.wait(max(0, next_t - time.perf_counter()))


def dry_run():
    from leader_teleop.buffer.data_sync_buffer import DataSyncBuffer

    data_sync_buffer = DataSyncBuffer(["eef_pose", "diffs"])

    # Start the Piper reader thread
    event_stop = threading.Event()
    piper_thread = threading.Thread(
        target=piper_reader_thread, args=(event_stop, data_sync_buffer)
    )
    piper_thread.start()

    # Start the Piper sender thread
    sender_thread = threading.Thread(
        target=piper_sender_thread, args=(event_stop, data_sync_buffer)
    )

    sender_thread.start()

    data_sync_buffer.get_buffer("diffs").add(
        {
            "joint_1": 0.0,
            "joint_2": 0.0,
            "joint_3": 0.0,
            "joint_4": 0.0,
            "joint_5": 0.0,
            "joint_6": 0.0,
        }
    )

    time.sleep(2)

    data_sync_buffer.get_buffer("diffs").add(
        {
            "joint_1": 0.0,
            "joint_2": 0.8,
            "joint_3": 0.8,
            "joint_4": 0.0,
            "joint_5": 0.0,
            "joint_6": 0.0,
        }
    )

    time.sleep(2)
    data_sync_buffer.get_buffer("diffs").add(
        {
            "joint_1": 0.0,
            "joint_2": 0.0,
            "joint_3": 0.0,
            "joint_4": 0.0,
            "joint_5": 0.0,
            "joint_6": 0.0,
        }
    )
    time.sleep(1)

    # Stop threads
    event_stop.set()
    piper_thread.join()
    sender_thread.join()

    print("Dry run completed.")


if __name__ == "__main__":
    dry_run()
