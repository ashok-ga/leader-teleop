import yaml
import time
from commands import DynamixelCommands
from dynamixel_sdk import *
from enum import Enum
from multiprocessing import Process, Queue

class JointName(Enum):
    JOINT_1 = 'joint_1'
    JOINT_2 = 'joint_2'
    JOINT_3 = 'joint_3'
    JOINT_4 = 'joint_4'
    JOINT_5 = 'joint_5'
    JOINT_6 = 'joint_6'
    GRIPPER = 'gripper'

class ServoConfig:
    def __init__(self, config_file):
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

    def get_motor_table(self, motor_type):
        return self.config['motor_tables'].get(motor_type, {})

    def get_robot_joints(self):
        return self.config['robot']['joints']

class Joint:
    def __init__(self, joint_cfg, motor_table, packet_handler, port_handler):
        self.name = joint_cfg['name']
        self.id = joint_cfg['id']
        self.motor_type = joint_cfg['motor_type']
        self.min = joint_cfg['min']
        self.max = joint_cfg['max']
        self.packet_handler = packet_handler
        self.port_handler = port_handler

        self.addr_torque_enable = motor_table['torque_enable']
        self.addr_goal_position = motor_table['goal_position']
        self.addr_present_position = motor_table['present_position']
        self.addr_present_current = motor_table.get('present_current', None)
        self.position_bytes = motor_table['position_bytes']

        DynamixelCommands.enable_torque(self.packet_handler, self.port_handler, self.id, self.addr_torque_enable)

    def move(self, position):
        clipped = max(self.min, min(self.max, position))
        DynamixelCommands.set_goal_position(
            self.packet_handler, self.port_handler, self.id,
            self.addr_goal_position, clipped, self.position_bytes
        )

    def read_position(self):
        pos, _, _ = DynamixelCommands.read_present_position(
            self.packet_handler, self.port_handler, self.id,
            self.addr_present_position, self.position_bytes
        )
        return pos

    def read_current(self):
        if self.addr_present_current is None:
            return None
        current, _, _ = DynamixelCommands.read_present_current(
            self.packet_handler, self.port_handler, self.id,
            self.addr_present_current
        )
        return current

    def shutdown(self):
        DynamixelCommands.disable_torque(self.packet_handler, self.port_handler, self.id, self.addr_torque_enable)

class RobotArm:
    def __init__(self, config_file, port='/dev/ttyUSB0', baudrate=1000000):
        self.config = ServoConfig(config_file)
        self.packet_handler = PacketHandler(2.0)
        self.port_handler = PortHandler(port)

        self.port_handler.openPort()
        self.port_handler.setBaudRate(baudrate)

        self.joints = []
        for joint_cfg in self.config.get_robot_joints():
            motor_table = self.config.get_motor_table(joint_cfg['motor_type'])
            joint = Joint(joint_cfg, motor_table, self.packet_handler, self.port_handler)
            self.joints.append(joint)

    def move_joint(self, joint_name_enum, position):
        name = joint_name_enum.value
        joint = next((j for j in self.joints if j.name == name), None)
        if joint:
            joint.move(position)

    def read_joint_states(self):
        return [(j.name, j.read_position(), j.read_current()) for j in self.joints]

    def shutdown(self):
        for joint in self.joints:
            joint.shutdown()
        self.port_handler.closePort()

def robot_control_loop(command_queue):
    robot = RobotArm('robot_config.yaml')
    try:
        while True:
            if not command_queue.empty():
                cmd = command_queue.get()
                if cmd == 'exit':
                    break
                joint_name, position = cmd
                robot.move_joint(joint_name, position)
            time.sleep(0.01)
    finally:
        states = robot.read_joint_states()
        for name, pos, cur in states:
            print(f"{name}: pos={pos}, current={cur}")
        robot.shutdown()

if __name__ == '__main__':
    command_queue = Queue()
    control_process = Process(target=robot_control_loop, args=(command_queue,))
    control_process.start()

    command_queue.put((JointName.JOINT_1, 1731))
    command_queue.put((JointName.GRIPPER, 512))

    time.sleep(1)
    command_queue.put('exit')
    control_process.join()
