from dynamixel_sdk import *

class DynamixelCommands:
    @staticmethod
    def enable_torque(packet_handler, port_handler, dxl_id, addr_torque_enable):
        return packet_handler.write1ByteTxRx(port_handler, dxl_id, addr_torque_enable, 1)

    @staticmethod
    def disable_torque(packet_handler, port_handler, dxl_id, addr_torque_enable):
        return packet_handler.write1ByteTxRx(port_handler, dxl_id, addr_torque_enable, 0)

    @staticmethod
    def set_goal_position(packet_handler, port_handler, dxl_id, addr_goal_position, position, bytes_):
        if bytes_ == 2:
            return packet_handler.write2ByteTxRx(port_handler, dxl_id, addr_goal_position, position)
        elif bytes_ == 4:
            return packet_handler.write4ByteTxRx(port_handler, dxl_id, addr_goal_position, position)

    @staticmethod
    def read_present_position(packet_handler, port_handler, dxl_id, addr_present_position, bytes_):
        if bytes_ == 2:
            return packet_handler.read2ByteTxRx(port_handler, dxl_id, addr_present_position)
        elif bytes_ == 4:
            return packet_handler.read4ByteTxRx(port_handler, dxl_id, addr_present_position)

    @staticmethod
    def read_present_current(packet_handler, port_handler, dxl_id, addr_present_current):
        return packet_handler.read2ByteTxRx(port_handler, dxl_id, addr_present_current)
