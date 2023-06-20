#!/bin/bash

python3 -m cProfile -o test.prof vision_core.py --usb 0 --model yolov4-dla0-tiny 
