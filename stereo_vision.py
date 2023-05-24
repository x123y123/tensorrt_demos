import numpy as np
import cv2

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

def stereo_calibration() -> None:
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


if __name__ == '__main__':
    stereo_calibration()
    #main()    
