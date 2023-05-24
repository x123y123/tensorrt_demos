"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""


import os
import time
import argparse

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver

from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO
from control import drone_controller
from kalman import MyKalman

WINDOW_NAME = 'TrtYOLODemo'

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


def loop_and_detect(cam, trt_yolo, conf_th, vis,img_width,img_height):
    """Continuously capture images from camera and do object detection.
    # Arguments
      cam: the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """
    kalman_start = time.time()
    connection_string = '/dev/ttyACM0'
    sleep_time = 0.5
    kf = MyKalman()
    drone = drone_controller(connection_string)
    #drone.takeoff(1.25)

    frame_cnt = 0
    full_scrn = False
    fps = 0.0
    tic = time.time()
    cnt = 0
#    while frame_cnt < 100:
    while True:
        frame_start = time.time()
        """
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break
        """
        img = cam.read()
        if img is None:
            break
        boxes, confs, clss = trt_yolo.detect(img, conf_th)
        img , bb_coor = vis.draw_bboxes(img, boxes, confs, clss)
        
        
        if (len(boxes) > 0):
            kalman_Y = time.time()
            dt = kalman_Y - kalman_start
            kalman_start = kalman_Y

            Area = 0 
            # calculate bbox center
            bb_center_w = (bb_coor[2] - bb_coor[0]) / 2 + bb_coor[0]
            bb_center_h = (bb_coor[3] - bb_coor[1]) / 2 + bb_coor[1]
            Area = (bb_coor[2] - bb_coor[0]) * (bb_coor[3] - bb_coor[1])
            #Area = Area / 100000
            print(f"Area: {Area}")
            t = open("/home/uav/code/tensorrt_demos/bb_dis_25.txt", "a+")
            t.write("25 " + str(Area))
            t.write("\n")
            
            # kalman filter
            dist = kf.bbox2dist(Area)
            dist_E = kf.update(dist, drone.vehicle.velocity[0], dt)
            kal = open("/home/uav/code/tensorrt_demos/kal_dist_test2.txt", "a+")
            kal.write(str(dist) + " " + str(dist_E) + "\n")
            kal.close()
            
            # calculate img center
            img_center_w = img_width / 2
            img_center_h = img_height / 2
            #print(f"bb_center_w: {bb_center_w}")
            #print(f"bb_center_h: {bb_center_h}")
            #print(f"img_center_w: {img_center_w}")
            #print(f"img_center_h: {img_center_h}")
            
            if ((bb_center_h - img_center_h) != 0):
                h_index = abs(bb_center_h - img_center_h)
                #print(f"height_index: {h_index}")
                if (bb_center_h < img_center_h):
                    #print("move up!")
                    z = -0.05

                elif (bb_center_h > img_center_h):
                    #print("move down!")
                    z = 0.05
            else:
                z = 0

            if ((bb_center_w - img_center_w) != 0):
                w_index = abs(bb_center_w - img_center_w)
                #print(f"width_index: {w_index}")
                if (bb_center_w < img_center_w):
                    #print("move left!")
                    y = -0.05

                elif (bb_center_w > img_center_w):
                    #print("move right")
                    y = 0.05
            else:
                y = 0

            depth = 0
            if (h_index < 400 and w_index < 400):
                if (Area > 7):
                    x = -0.05
                    
                    #drone.move_backward()
                    f = open("/home/uav/code/tensorrt_demos/depth.txt", "r")
                    depth_str = f.read()
                    f.close()
                    try:
                        depth = int(depth_str)
                    except ValueError:
                        print("some_variable did not contain a number!")
                    if (depth < 30):
                        print("[stereo] move back")
                        x = -0.05
                    else:
                        print("\033[32m [stereo] \033[0m move forward")
                        x = 0.05
                    
                    
                elif (Area < 7):
                    #drone.move_forward()
                    x = 0.05
        else:
            print("\033[31m No object detect \033[0m")
            x = 0
            y = 0
            z = 0
        
        if (cnt == 5):
            #drone.move(x, y, z)
            #time.sleep(sleep_time)
            cnt = 0
            #print("move")
        cnt = cnt + 1
        #writer.write(img)
        frame_end = time.time()
        frame_time = frame_end - frame_start
        #print(f"per frame:{frame_time}")

        """ 
        img = show_fps(img, fps)
        cv2.imshow(WINDOW_NAME, img)
        toc = time.time()
        curr_fps = 1.0 / (toc - tic)
        # calculate an exponentially decaying average of fps number
        #fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
        fps = curr_fps
        tic = toc             
        key = cv2.waitKey(1)
        if key == 27:  # ESC key: quit program
            break
        elif key == ord('F') or key == ord('f'):  # Toggle fullscreen
            full_scrn = not full_scrn
            set_display(WINDOW_NAME, full_scrn)
        """
        frame_cnt = frame_cnt + 1 
        

def main():
    
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)
    

    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    """
    open_window(
        	WINDOW_NAME, 
        	'Camera TensorRT YOLO Demo',
        	cam.img_width, cam.img_height
        	)
    """
    loop_and_detect(cam,trt_yolo,args.conf_thresh,vis=vis, img_width = cam.img_width, img_height = cam.img_height)
    #writer.release()
    cam.release()
    #distance.join()
    print('Done.')
    
    #cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main()
