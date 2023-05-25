import numpy as np
import jetson.inference
import jetson.utils
import time

baseline = 60 #mm
focal = 2.6   #mm
alpha = 73 
bias = 3
threshold = 10

def find_depth():
    cnt = 0
    max_area_det = 0.0
    width_bb = 0.0
    height_bb = 0.0
    
    net = jetson.inference.detectNet(
                                     model = "model/model0114_retrain/ssd-mobilenet.onnx", 
                                     labels = "model/model0114_retrain/labels.txt", 
                                     input_blob = "input_0", 
                                     output_cvg = "scores", 
                                     output_bbox = "boxes", 
                                     #                                     threshold = 0.4
                                    )
    input0 = jetson.utils.videoSource("csi://0")      # '/dev/video0' for V4L2
    input1 = jetson.utils.videoSource("csi://1")      # '/dev/video0' for V4L2
    display = jetson.utils.videoOutput("display://0") # 'my_video.mp4' for file
    #display = jetson.utils.videoOutput("/home/uav/code/mango_drone/test.mp4") # 'my_video.mp4' for file

    #time.sleep(1)
    frame_cnt = 0
#    while True:
    while frame_cnt < 100:
        img0 = input0.Capture()
        detections0 = net.Detect(img0)
        img1 = input1.Capture()
        detections1 = net.Detect(img1)
            
        imgcenter_width0 = img0.width / 2
        imgcenter_height0 = img0.height / 2
        imgcenter_width1 = img1.width / 2
        imgcenter_height1 = img1.height / 2
        max_area_det = 0.0
        width_bb0 = 0.0
        height_bb0 = 0.0
        width_bb1 = 0.0
        height_bb1 = 0.0
            
        for detection in detections0:
            width_bb0 = detection.Center[0]
            height_bb0 = detection.Center[1]
            #if detection.Area > max_area_det:
            #max_area_det = detection.Area                                       
            
        for detection in detections1:
            width_bb1 = detection.Center[0]
            height_bb1 = detection.Center[1]
                    
        if (imgcenter_width0 == imgcenter_width1):
            #f_pixel = (img0.width * 0.5) / np.tan(alpha * 0.5 * np.pi/180)
            pix2mm = 0.00112 
        else:
            print('Left and right camera frames do not have the same pixel width')
    
        disparity = abs(width_bb0 - width_bb1)
    
        if (width_bb0 and width_bb1):
            if (disparity == 0):
                print('disparity is zero')
            else:
                #zDepth = int((20 / 7) * ((baseline * f_pixel) / disparity) - bias)
                zDepth = ((baseline * focal) / (disparity * pix2mm * 10)) 
                #print(f"0: ({width_bb0}, {height_bb0})")
                #print(f"1: ({width_bb1}, {height_bb1})")
                print(zDepth)
                frame_cnt = frame_cnt + 1
                #t = open("/home/uav/code/tensorrt_demos/ste_dis_225.txt", "a+")
                #t.write("225 " + str(zDepth))
                #t.write("\n")
                f = open("/home/uav/code/tensorrt_demos/depth.txt", "w")
                f.write(str(zDepth))
                """
                f.write("\n")
                cnt = cnt + 1
                if cnt % threshold == 0:
                    cnt = 0
                    f.seek(0)
                    f.truncate()
                """
                f.close()
    #        jetson.utils.cudaDrawCircle(img, (width_bb, height_bb), 10, (255, 0, 0))
    #        jetson.utils.cudaDrawCircle(img, (img.width/2, img.height/2), 10, (255, 0, 0))
            display.Render(img0)
            # exit on input EOS
            if not input0.IsStreaming():
                break
    
    print("program finish!")
    
if __name__ == '__main__':
    find_depth()
