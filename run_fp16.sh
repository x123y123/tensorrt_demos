#!/bin/bash

python3 -m cProfile -o test.prof vision_core.py --usb 0 --model fake_mango_3000pics/yolov4-tiny 
