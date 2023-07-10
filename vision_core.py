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
OPEN_KALMAN = True
SAVE_KF = False
POWER_RECORD = True
x_speed = 0
y_speed = 0
z_speed = 0
x = 0
y = 0
z = 0
sleep_time = 0.01
threshold = 2500
cpu_freq = 1907200

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
    
    if (OPEN_KALMAN):
        kalman_start = time.time()
        kf = MyKalman(mode = 1)
    
    
    connection_string = '/dev/ttyACM0'
    connect_time_s = time.time()
    drone = drone_controller(connection_string)
    connect_time_e = time.time()
    connect_time = connect_time_e - connect_time_s
    
    if (REAL_FLY):
        drone.takeoff(1.25)
    
    frame_perf = []
    sleep_perf = []
    fps_perf = []
    frame_cnt = 0
    detect_cnt = 0
    full_scrn = False
    fps = 0.0
    tic = time.time()
    total_power = 0.0
    cnt = 0
    total_time_s = time.time()

    #data_stamp = open("/home/uav/code/tensorrt_demos/tony/forward_time/ex1.txt", "a+")
    
    if (SAVE_KF):
        kf_save = open("/home/uav/code/tensorrt_demos/david/kf_data_0630_1.txt", "a+")
        kf_save.write('Area vx vy dt lat lon pixel_w yaw\n')
        kf_save.close()

    #while detect_cnt < 2000:
    #while frame_cnt < 100:
    while True:
        global threshold
        global x
        global y
        global z
        global x_speed
        global y_speed
        
        frame_start = time.time()
        
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
            detect_cnt += 1

            # calculate img center
            img_center_w = img_width / 2
            img_center_h = img_height / 2
            # calculate bbox center
            bb_center_w = (bb_coor[2] - bb_coor[0]) / 2 + bb_coor[0]
            bb_center_h = (bb_coor[3] - bb_coor[1]) / 2 + bb_coor[1]
            Area = (bb_coor[2] - bb_coor[0]) * (bb_coor[3] - bb_coor[1])
            #Area = Area / 100000
            print(f"Area: {Area}")
       
            # kalman filter
            if (OPEN_KALMAN):
                dist = round(kf.bbox2dist(Area), 2)
                """
                #calc dt
                kalman_Y = time.time()
                dt = kalman_Y - kalman_start
                kalman_start = kalman_Y
                
                #test 3D kalman
                vel = np.array([drone.vehicle.velocity[0], drone.vehicle.velocity[1]])
                data = kf.update(Area, vel, dt, pixel = bb_center_w - img_center_w)
                print('mango coor: x: ' + str(data[1]) + ' y: ' + str(data[2]) + '\n')
                """
                #save drone state in file
                if (SAVE_KF):
                    lat = drone.vehicle.location.global_frame.lat
                    lon = drone.vehicle.location.global_frame.lon
                    yaw = drone.vehicle.attitude.yaw
                    kf_save = open("/home/uav/code/tensorrt_demos/david/kf_data_0630_1.txt", "a+")
                    kf_save.write(str(Area) + ' ' + str(vel[0]) + ' ' + str(vel[1]) + ' ' + str(dt) + ' ' + str(lat) + ' ' + str(lon) + ' ' + str(bb_center_w - img_center_w) + ' ' + str(yaw) + '\n')
                    kf_save.close()
                    print('Data saved !')
                
                #put area into kalman
                if (PUT_TEXT):
                    text = "kalman dist: " + str(dist)
                    cv2.putText(
                                img, 
                                text, 
                                (0, 320), 
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1, 
                                (0, 255, 255), 
                                1, 
                                cv2.LINE_AA
                            )
            if (dist > 150): 
                threshold = 8100
                x_speed = 0.2
                y_speed = 0.09
                z_speed = 0.095

            elif (dist <= 150 and dist > 130):
                threshold = 6400
                x_speed = 0.2
                y_speed = 0.09
                z_speed = 0.095

            elif (dist <= 130 and dist > 110):
                threshold = 6400
                x_speed = 0.2
                y_speed = 0.09
                z_speed = 0.095

            elif (dist <= 110 and dist > 90):
                threshold = 4900
                x_speed = 0.2
                y_speed = 0.08
                z_speed = 0.085

            elif (dist <= 90 and dist > 70):
                threshold = 4900
                x_speed = 0.2
                y_speed = 0.08
                z_speed = 0.085

            elif (dist <= 70 and dist > 40):
                threshold = 3600
                x_speed = 0.2
                y_speed = 0.07
                z_speed = 0.075

            else:
                threshold = 1600
                x_speed = 0.2
                y_speed = 0.06
                z_speed = 0.065

            if (VIDEO_STREAM):
                #draw threshold
                cv2.circle(img, (320, 240), int(threshold**0.5), (255, 0, 0), 2)
            
            
            if ((bb_center_h - img_center_h)**2 + (bb_center_w - img_center_w)**2) > threshold:
                x = 0
                if (bb_center_h < img_center_h):
                    if (PUT_TEXT):
                        cv2.putText(
                                img, 
                                "move up", 
                                (0, 360), 
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1, 
                                (0, 255, 255), 
                                1, 
                                cv2.LINE_AA
                                )
                    print("move up!")
                    z = -z_speed

                elif (bb_center_h > img_center_h):
                    if (PUT_TEXT):
                        cv2.putText(
                                img, 
                                "move down", 
                                (0, 360), 
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1, 
                                (0, 255, 255), 
                                1, 
                                cv2.LINE_AA
                                )
                    print("move down!")
                    z = z_speed

                if (bb_center_w < img_center_w):
                    if (PUT_TEXT):
                        cv2.putText(
                                img, 
                                "move left", 
                                (0, 400), 
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1, 
                                (0, 255, 255), 
                                1, 
                                cv2.LINE_AA
                                )
                    print("move left!")
                    y = -y_speed

                elif (bb_center_w > img_center_w):
                    if (PUT_TEXT):
                        cv2.putText(
                                img, 
                                "move right", 
                                (0, 400), 
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1, 
                                (0, 255, 255), 
                                1, 
                                cv2.LINE_AA
                                )
                    print("move right")
                    y = y_speed
            else:
                if (bb_center_h > img_center_h):
                    if (PUT_TEXT):
                        cv2.putText(
                                img, 
                                "move down", 
                                (0, 360), 
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1, 
                                (0, 255, 255), 
                                1, 
                                cv2.LINE_AA
                                )
                    print("move down!")
                    z = z_speed
                else:
                    if (PUT_TEXT):
                        cv2.putText(
                                    img, 
                                    "move forward", 
                                    (0, 440), 
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1, 
                                    (0, 255, 255), 
                                    1, 
                                    cv2.LINE_AA
            
                                )
                    print("move forward")
                    x = x_speed 
                    y = 0
                    z = 0
        
            if (OPEN_KALMAN):
                if (dist < 25):     # modify 25~20
                    if (PUT_TEXT):
                        cv2.putText(
                                    img, 
                                    "move backward", 
                                    (0, 440), 
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1, 
                                    (0, 255, 255), 
                                    1, 
                                    cv2.LINE_AA
                                )
                    print("move back")
                    x = -0.05
                    y = 0
                    z = 0
                    forward_end = time.time()
                    time_data = forward_end - tic
                    #data_stamp.write(str(threshold) + ' ' + str(time_data))
                    #data_stamp.close()

            else:
                if (Area > 55000):
                    if (PUT_TEXT):
                        cv2.putText(
                                    img, 
                                    "move backward", 
                                    (0, 440), 
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1, 
                                    (0, 255, 255), 
                                    1, 
                                    cv2.LINE_AA
                                )
                    print("move back")
                    x = -0.05
                    y = 0
                    z = 0
        else:
            print("\033[31m No object detect \033[0m")
            x = -0.05
            y = 0
            z = 0
       
            
        # print velocity message on img
        v_value = "[" + str(x) + ", " + str(y) + ", " + str(z) + "]"
        if (PUT_TEXT):
            cv2.putText(
                        img, 
                        v_value, 
                        (0, 280), 
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, 
                        (0, 255, 255), 
                        1, 
                        cv2.LINE_AA
                        )
        if (REAL_FLY):
            drone.move(x, y, z)
        
        print(f"x: {x} y: {y} z: {z}")
        
        #test sleep time
        sleep_s = time.time()
        time.sleep(sleep_time)
        sleep_e = time.time()
        sleep_b = sleep_e - sleep_s
        sleep_perf.append(sleep_b)

        total_power += (read_power() * sleep_time)

        cnt = cnt + 1
        writer.write(img)
        frame_end = time.time()
        frame_time = frame_end - frame_start
        print(f"per frame:{frame_time}")
        frame_perf.append(frame_time)
        
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
            
        frame_cnt = frame_cnt + 1

    total_time_e = time.time()
    total_time = total_time_e - total_time_s
    if(POWER_RECORD):
        r_cpu_freq = open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "r")
        cfreq = int(r_cpu_freq.read())

        p_file = open("/home/uav/code/tensorrt_demos/tony/power/total.txt", "a+")
#        p_file.write(str(cfreq) + ' ' + str(total_power) + ' ' + str(sum(frame_perf)/len(frame_perf)) + ' ' + str(sum(sleep_perf)/len(sleep_perf)) + ' ' + str(sum(fps_perf)/len(fps_perf)) + ' ' + str(total_time) + ' ' + str(connect_time) + ' ' + str(frame_cnt) + '\n')
        p_file.write(str(cfreq) + ' ' + str(total_power) + ' ' + str(sum(frame_perf)/len(frame_perf)) + ' ' + str(sum(sleep_perf)/len(sleep_perf)) + ' ' + str(total_time) + ' ' + str(connect_time) + ' ' + str(frame_cnt) + '\n')
    print(f"total power:{total_power}")
        

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
