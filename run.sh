#!/bin/bash

rm kf_data_0309_1.txt
python3 trt_yolo_copy_v3.py --usb 2 --model fake_mango_3000pics/yolov4-tiny
