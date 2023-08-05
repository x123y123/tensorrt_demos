"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""

import os
import time
import argparse
import numpy as np
import math

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver

from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO
from control import drone_controller
from kalman import MyKalman
from performance_model import *

WINDOW_NAME = 'TrtYOLODemo'
REAL_FLY = True
PUT_TEXT = False
VIDEO_STREAM = False
x_speed = 0
y_speed = 0
z_speed = 0
# pd parameters, index 0 for y, index 1 for z
kp = [0.000833, 0.000625]
kd = [0.000030, 0.000020]
pre_dw = 0.0
pre_dh = 0.0
sleep_time = 0.01
threshold = 2500
cpu_freq = 1907200

def bbox2dist(bbox) -> float:
    n = 3.180226752
    l0 = 1903.809012

    return (n * (l0 / (bbox ** 0.5) - 1))

def check_limit(value) -> float:
    if (value > 0.3):
        value = 0.3
    elif (value < -0.3):
        value = -0.3

    return value

def print_move_in_video(img, move: str, spot = (10, 400)):
    cv2.putText(
            img, 
            move, 
            spot, 
            cv2.FONT_HERSHEY_SIMPLEX,
            1, 
            (0, 255, 0), 
            1, 
            cv2.LINE_AA
            )
    return

def parse_args():
    """Parse input arguments."""
    desc = ('Capture and display live camera video, while doing '
            'real-time object detection with TensorRT optimized '
            'YOLO model on Jetson')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument(
        '-c', '--category_num', type=int, default=80,
        help='number of object categories [80]')
    parser.add_argument(
        '-t', '--conf_thresh', type=float, default=0.3,
        help='set the detection confidence threshold')
    
    parser.add_argument(
        '-m', '--model', type=str, required=True,
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish|yolov4-p5]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    args = parser.parse_args()
    return args


def loop_and_detect(cam, trt_yolo, conf_th, writer, vis,img_width,img_height):
    """Continuously capture images from camera and do object detection.
    # Arguments
      cam: the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """
    connection_string = '/dev/ttyACM0'
    drone = drone_controller(connection_string)
    
    if (REAL_FLY):
        drone.takeoff(1.25)
    
    fps_perf = []
    full_scrn = False
    fps = 0.0
    tic = time.time()
    total_time_s = time.time()
    
    while True:
        global threshold
        global x_speed
        global y_speed
        global z_speed
        global kp
        global kd
        global pre_dw
        global pre_dh
        
        if (VIDEO_STREAM):
            if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
                break
        
        img = cam.read()
        if img is None:
            break
        print("img work")
        boxes, confs, clss = trt_yolo.detect(img, conf_th)
        img , bb_coor = vis.draw_bboxes(img, boxes, confs, clss)
        Area = 0 
        dist = 0

        
        if (len(boxes) > 0):

            # calculate img center
            img_center_w = img_width / 2
            img_center_h = img_height / 2
            # calculate bbox center
            bb_center_w = (bb_coor[2] - bb_coor[0]) / 2 + bb_coor[0]
            bb_center_h = (bb_coor[3] - bb_coor[1]) / 2 + bb_coor[1]
            dw = bb_center_w - img_center_w
            dh = bb_center_h - img_center_h
            Area = (bb_coor[2] - bb_coor[0]) * (bb_coor[3] - bb_coor[1])
            #Area = Area / 100000
            print(f"Area: {Area}")
            dist = round(bbox2dist(Area), 2)
            print(f"Dist: {dist}")

            # test PD controller ===============================================
            y_speed = kp[0] * dw + kd[0] * (dw - pre_dw)
            z_speed = kp[1] * dh + kd[1] * (dh - pre_dh)
            y_speed = round(check_limit(y_speed), 2)
            z_speed = round(check_limit(z_speed), 2)

            pre_dw = dw
            pre_dh = dh

            # ==================================================================
            
            if (dist > 150): 
                threshold = 8100
                x_speed = 0.25

            elif (dist <= 150 and dist > 130):
                threshold = 6400
                x_speed = 0.25

            elif (dist <= 130 and dist > 110):
                threshold = 6400
                x_speed = 0.25

            elif (dist <= 110 and dist > 90):
                threshold = 4900
                x_speed = 0.2

            elif (dist <= 90 and dist > 70):
                threshold = 4900
                x_speed = 0.2

            elif (dist <= 70 and dist > 40):
                threshold = 3600
                x_speed = 0.2

            else:
                threshold = 1600
                x_speed = 0.2

            if (VIDEO_STREAM):
                #draw threshold
                cv2.circle(img, (320, 240), int(threshold**0.5), (255, 0, 0), 2)
            
            if (dh > 0):
                print_move_in_video(img, "down")
                print("move down")
            else:
                print_move_in_video(img, "up")
                print("move up")
            if (dw > 0):
                print_move_in_video(img, "right", spot = (10, 430))
                print("move right")
            else:
                print_move_in_video(img, "left", spot = (10, 430))
                print("move left")
            
            if (dist < 25):     # modify 25~20
                print_move_in_video(img, "move backward", spot = (10, 460))
                print("move back")
                x = -0.05
                y_speed = 0
                z_speed = 0
            elif (dh**2 + dw**2) < threshold:
                print_move_in_video(img, "move forward", spot = (10, 460))
                print("move forward")
                x = x_speed 
            else:
                x = 0
        
        else:
            print("\033[31m No object detect \033[0m")
            x = -0.05
            y = 0
            z = 0

        v_text = "[x: " + str(x) + " , y: " + str(y_speed) + " , z: " + str(z_speed) + "]"
        if (PUT_TEXT):
            cv2.putText(
                        img,
                        v_text,
                        (0, 280),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA
                        )
            
        # print velocity message on img
        if (REAL_FLY):
            drone.move(x, y_speed, z_speed)
        
        print(f"x: {x} y: {y_speed} z: {z_speed}")
        
        time.sleep(sleep_time)

        writer.write(img)
        
        if (VIDEO_STREAM):
            img = show_fps(img, fps)
            cv2.imshow(WINDOW_NAME, img)
            toc = time.time()
            curr_fps = 1.0 / (toc - tic)
            # calculate an exponentially decaying average of fps number
            fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
            #fps = curr_fps
            tic = toc             
            key = cv2.waitKey(1)
            if key == 27:  # ESC key: quit program
                break
            elif key == ord('F') or key == ord('f'):  # Toggle fullscreen
                full_scrn = not full_scrn
                set_display(WINDOW_NAME, full_scrn)
            fps_perf.append(fps)

def main():
    
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)
    

    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    frame_width = 640
    frame_height = 480

    writer = cv2.VideoWriter(
                '/home/uav/code/tensorrt_demos/result.mp4',
                cv2.VideoWriter_fourcc(*'mp4v'),
                10,
                (frame_width, frame_height),
                )


    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)
    
    if (VIDEO_STREAM):
        open_window(
            	WINDOW_NAME, 
            	'Camera TensorRT YOLO Demo',
                cam.img_width, cam.img_height
                )
    
    #set_freq(cpu_freq)
    loop_and_detect(
            cam,trt_yolo,
            args.conf_thresh,
            writer = writer,
            vis=vis, 
            img_width = cam.img_width, 
            img_height = cam.img_height
            )
    writer.release()
    cam.release()
    #distance.join()
    print('Done.')
    
    if (VIDEO_STREAM):
        cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main()
