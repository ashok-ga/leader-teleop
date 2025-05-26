#!/bin/bash
set -e

# 1. Activate CAN interface
bash can_activate.sh can0 1000000

# 2. Run piper_joint_control.py
python3 piper_ctrl_joint.py

# 3. Run data_rec_newest.py
python3 data_recorder.py
