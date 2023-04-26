import numpy as np
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
"""
from control import drone_controller
"""
WINDOW_NAME = 'TrtYOLODepth'
MODEL_PATH = 'yolo/fake_mango_3000pics/yolo_v4-tiny'
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

def gstreamer_pipeline(
        sensor_id,
        sensor_mode=3,
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=30,
        flip_method=0,
):
    return (
            "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                sensor_mode,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )

def take_pic() -> None:
    """
    Function: take_pic()
    --------------------
    Using 
    return: None
    """
    cam0 = cv2.VideoCapture( 
    	                    gstreamer_pipeline(sensor_id=0), 
                            cv2.CAP_GSTREAMER
                            )
    	
    cam1 = cv2.VideoCapture( 
    	                    gstreamer_pipeline(sensor_id=1),
                            cv2.CAP_GSTREAMER
                            )

    if not cam0.isOpened():
        print("Failed to open cam0")
    if not cam1.isOpened():
        print("Failed to open cam1")
    
    num = 0
    
    while cam0.isOpened():
        success0, frame0 = cam0.read()
        success1, frame1 = cam1.read()
        
        k = cv2.waitKey(2) 
        
        if k == 27:
            break
        elif k == ord('s'):
            cv2.imwrite('imgL' + str(num) + '.png', frame0)
            cv2.imwrite('imgR' + str(num) + '.png', frame1)
            print('img saved')
            num += 1
        
        cv2.imshow('ImgL', frame0)
        cv2.imshow('ImgR', frame1)

def calibration_img() -> None:
    chessboard_size = (8, 6)
    frame_size = (640, 480)

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1,2)

    # Arrays to store obj points
    objpoints = []
    imgpointsL = []
    imgpointsR = []

    
    img_left_o = cv2.imread('imgL0.png')
    img_right_o = cv2.imread('imgR0.png')
    img_left = cv2.imread('imgL0.png', cv2.IMREAD_GRAYSCALE)
    img_right = cv2.imread('imgR0.png', cv2.IMREAD_GRAYSCALE)

    # find corner
    retL, cornersL = cv2.findChessboardCorners(img_left, chessboard_size, None)
    retR, cornersR = cv2.findChessboardCorners(img_right, chessboard_size, None)

    cv2.imshow('o',img_left)
    if retL and retR == True:
        objpoints.append(objp)
        cornersL = cv2.cornerSubPix(img_left, cornersL, (5, 5), (-1, -1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv2.cornerSubPix(img_right, cornersR, (5, 5), (-1, -1), criteria)
        imgpointsR.append(cornersR)

        # Draw and display
        cv2.drawChessboardCorners(img_left, chessboard_size, cornersL, retL)
        cv2.drawChessboardCorners(img_right, chessboard_size, cornersR, retR)

        cv2.imshow('imgL', img_left)
        cv2.imshow('imgR', img_right)

        cv2.waitKey(1000)
        cv2.destroyAllWindows()
    
    ###### calibration ######
    retL, cameraMatrixL, distL, revcsL, tvecsL = cv2.calibrateCamera(
                                                                    objpoints, 
                                                                    imgpointsL, 
                                                                    frame_size, 
                                                                    None, 
                                                                    None
                                                                    )
    print(img_left_o.shape)
    heightL, widthL, channelsL = img_left_o.shape
    newCameraMatrixL, roi_L = cv2.getOptimalNewCameraMatrix(
                                                            ameraMatrixL, 
                                                            distL, 
                                                            (widthL, heightL), 
                                                            1, 
                                                            (widthL, heightL)
                                                            )

    retR, cameraMatrixR, distR, revcsR, tvecsR = cv2.calibrateCamera(
                                                                     bjpoints, 
                                                                     imgpointsR, 
                                                                     frame_size, 
                                                                     None, 
                                                                     None
                                                                     )
    heightR, widthR, channelsR = img_right_o.shape
    newCameraMatrixR, roi_R = cv2.getOptimalNewCameraMatrix(
                                                            cameraMatrixR, 
                                                            distR, 
                                                            (widthR, heightR), 
                                                            1, 
                                                            (widthR, heightR)
                                                            )

    ##### stereo vision calibration ####

    flags = 0
    flags |= cv2.CALIB_FIX_INTRINSIC

    criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    res_stereo, newCameraMatrixL, distL, newCameraMatrixR, distR, \
    rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(
                                                                         objpoints, 
                                                                         imgpointsL, 
                                                                         imgpointsR, 
                                                                         newCameraMatrixL, 
                                                                         distL, 
                                                                         newCameraMatrixR, 
                                                                         distR, 
                                                                         img_left.shape[::-1], 
                                                                         criteria_stereo, 
                                                                         flags
                                                                         )

    #### stereo rectification ###
    rectifyScale = 1
    rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R = cv2.stereoRectify(
                                                                                newCameraMatrixL, 
                                                                                distL, 
                                                                                newCameraMatrixR, 
                                                                                distR, 
                                                                                img_right.shape[::-1], 
                                                                                rot, 
                                                                                trans, 
                                                                                rectifyScale, 
                                                                                (0, 0)
                                                                                )
    
    stereoMapL = cv2.initUndistortRectifyMap(
                                             newCameraMatrixL, 
                                             distL, 
                                             rectL, 
                                             projMatrixL, 
                                             img_left.shape[::-1], 
                                             cv2.CV_16SC2
                                             )

    stereoMapR = cv2.initUndistortRectifyMap(
                                             newCameraMatrixR, 
                                             distR, 
                                             rectR, 
                                             projMatrixR, 
                                             img_right.shape[::-1], 
                                             cv2.CV_16SC2
                                             )
    print('Saving parameters!')
    cv_file = cv2.FileStorage('stereoMap.xml', cv2.FILE_STORAGE_WRITE)

    cv_file.write('stereoMapL_x', stereoMapL[0])
    cv_file.write('stereoMapL_y', stereoMapL[1])
    cv_file.write('stereoMapR_x', stereoMapR[0])
    cv_file.write('stereoMapR_y', stereoMapR[1])

    cv_file.release()

def stereo_calibration(frameR, frameL):
    cv_file = cv2.FileStorage()
    cv_file.open(stereoMap.xml, cv2.FileStorage_READ)
    sereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
    sereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
    sereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
    sereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

    undistortedL = cv2.remap(frameL, stereoMapL_x, stereoMap_y, cv2.INTER_LANCZOS4)
    undistortedR = cv2.remap(frameL, stereoMapL_x, stereoMap_y, cv2.INTER_LANCZOS4)
    
    return unistortedR, unistortedL

def find_depth(right_point, left_point, frame_right, frame_left, baseline, f, alpha):
    height_right, width_right, depth_right = frame_right.shape
    height_left, width_left, depth_left = frame_left.shape

    if width_right == width_left:
        f_pixel = (width_right * 0.5) / np.tan(alpha * 0.5 * np.pi/180)

    else:
        print('Left and right camera frames do not have the same pixel width')

    x_right = right_point[0]
    x_left = left_point[0]

    # calculate the disparity
    disparity = x_left - x_right

    #calculate the depth z (cm)
    zDepth = (baseline * f_pixel) / disparity 

    return abs(zDepth)

def stereo_depth_estimation():
    
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)


    # open cam
    cam0 = cv2.VideoCapture( 
    	                    gstreamer_pipeline(sensor_id=0), 
                            cv2.CAP_GSTREAMER
                            )
    	
    cam1 = cv2.VideoCapture( 
    	                    gstreamer_pipeline(sensor_id=1),
                            cv2.CAP_GSTREAMER
                            )

    if not cam0.isOpened():
        print("Failed to open cam0")
    if not cam1.isOpened():
        print("Failed to open cam1")
    
    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)
    frame_width0, frame_height0 = int(cam0.get(3)), int(cam0.get(4))
    frame_width1, frame_height1 = int(cam1.get(3)), int(cam1.get(4))
    writer0 = cv2.VideoWriter(
    				#args.output,
    				'result_dep.mp4',
    				cv2.VideoWriter_fourcc(*'mp4v'), 
    				30, 
    				(frame_width0, frame_height0)
    			      )
    writer1 = cv2.VideoWriter(
    				#args.output,
    				'result_dep.mp4',
    				cv2.VideoWriter_fourcc(*'mp4v'), 
    				30, 
    				(frame_width1, frame_height1)
    			      )
    
    # stereo vision setup parameters
    frame_rate = 30     # camera frame rate (max at 30 fps, in 3280x2464 resolution)
    B = 6               # baseline (cm)
    F = 2.6             # camera lense's focal length (mm)
    alpha = 73          # camera field of view in horizontal plane (degrees)
    
    while (cam0.isOpened() and cam1.isOpened()):
        success_right, frame_right = cam0.read()
        success_left, frame_left = cam1.read()

        # calibration
        frame_right, frame_left = stereo_calibration(frame_right, frame_left)

        ############# 
        if not success_right or not success_left:
            break

        else:
            start = time.time()
            # convert the image BGR to RGB
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)
            frame_right , bb_coorR = vis.draw_bboxes(img, boxes, confs, clss)
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)
            frame_left , bb_coorL = vis.draw_bboxes(img, boxes, confs, clss)
            
            # bbox center
            bb_center_wR = (bb_coorR[2] - bb_coorR[0]) / 2 + bb_coorR[0]
            bb_center_hR = (bb_coorR[3] - bb_coorR[1]) / 2 + bb_coorR[1]
            
            bb_center_wL = (bb_coorL[2] - bb_coorL[0]) / 2 + bb_coorL[0]
            bb_center_hL = (bb_coorL[3] - bb_coorL[1]) / 2 + bb_coorL[1]

            # find mango 
            boxesR, confsR, clssR = trt_yolo.detect(frame-right, 0.3)
            boxesR, confsR, clssR = trt_yolo.detect(frame_left, 0.3)
            # convert the RGB to BGR 
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_RGB2BGR)
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_RGB2BGR)

            ####### calculate depth #######
            center_left = 0
            center_right = 0
            if len(boxesR) < 0 or len(boxesL):
                pass

            else:
                depth = find_depth((bb_center_wR, bb_center_hR), (bb_center_wL, bb_center_hL), frame_right, frame_left, B, F, alpha)
                print(f"Depth: {depth}")
                

    writer0.release()
    writer1.release()
    cam0.release()
    cam1.release()

    
if __name__ == '__main__':
    take_pic()
    #stereo_depth_estimation()
    #main()    
