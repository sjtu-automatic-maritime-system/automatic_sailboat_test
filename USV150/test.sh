#!/bin/bash

python AHRS.py &
python GNSS.py &
python Voltage.py &
python motor_tlg.py