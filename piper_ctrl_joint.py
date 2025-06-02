#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
from typing import (
    Optional,
)
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()

    piper.EnableArm()
    # while( not piper.EnablePiper()):
    #     time.sleep(0.01)
    factor = 57295.7795  # 1000*180/3.1415926
    # factor =1
    position = [0, 0, 0, 0, 0, 0, 0]

    joint_0 = round(position[0] * factor)
    joint_1 = round(position[1] * factor)
    joint_2 = round(position[2] * factor)
    joint_3 = round(position[3] * factor)
    joint_4 = round(position[4] * factor)
    joint_5 = round(position[5] * factor)
    joint_6 = round(position[6] * 1000 * 1000)
    # piper.MotionCtrl_1()
    piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    print("Piper Arm Enabled")
    time.sleep(0.005)
    pass
