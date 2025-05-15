from typing import (
    Optional,
)
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can_right")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    # 3rad/s
    for i in range(1,7):
        piper.MotorMaxSpdSet(i, 3000)
        time.sleep(0.1)
    while True:
        piper.SearchAllMotorMaxAngleSpd()
        print(piper.GetAllMotorAngleLimitMaxSpd())
        time.sleep(0.01)