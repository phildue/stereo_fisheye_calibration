  
from camera import StereoCam
from image_collection import load_images,Checkerboard,find_corners
from globals import CalibrationException,checkerboard,square_size,image_resolution,max_frames
import argparse

import os
import cv2
import numpy as np

checkerboard = Checkerboard(checkerboard,square_size) # The object we want to find
stereo_rig = StereoCam("stereopi",image_resolution) # The camera we want to calibrate

parser = argparse.ArgumentParser()
parser.add_argument('--source_left',
                    help='Select video device to directly fetch images from camera or folder to load pre recorded images.'
                          'In case of live calibration images are stored within ./calibration_images. '
                          'Otherwise source should contain glob expression a la "calibration_images/left*.png"',
                    default="/dev/video0",
                    )
parser.add_argument('--source_right',
                    help='Select "live" to directly fetch images from camera or folder to load pre recorded images.'
                          'In case of live calibration images are stored within ./calibration_images. '
                          'Otherwise source should contain glob expression a la "calibration_images/right*.png"',
                    default="/dev/video1",
                    )
parser.add_argument('--iterative', dest='iterative', action='store_true',help="If set attempt to calibrate after each frame to make sure images are useful.")
parser.add_argument('--max_frames', help='Choose how many frames should be used for calibration', default=max_frames,type=int)

args = parser.parse_args()
live_mode = args.source_left.startswith('/dev') or args.source_right.startswith('/dev')
corners_l = [] # 2d points in image plane left camera.
corners_r = [] # 2d points in image plane right camera.

fNo = 0
for grayL,grayR in load_images(args.source_left,args.source_right,image_resolution):
    loadedY, loadedX  =  grayL.shape

    try:
        corners_r_img  = find_corners(grayR,"Right",checkerboard)
        corners_l_img  = find_corners(grayL,"Left",checkerboard)

        if args.iterative:
            corners_r_tmp = corners_r.copy()
            corners_l_tmp = corners_l.copy()
            rmse,rmse_l,rmse_r = stereo_rig.calibrate([checkerboard._p3d]*len(corners_l),corners_l,corners_r)
            print ("Calibrated:\n{}".format(stereo_rig))

        if live_mode:
            cv2.imwrite("./calibration_images/left_{}.png".format(fNo),grayL)
            cv2.imwrite("./calibration_images/right_{}.png".format(fNo),grayR)
        

        corners_l.append(corners_l_img)
        corners_r.append(corners_r_img)
        fNo += 1
        if fNo > args.max_frames:
            break
    except CalibrationException:
        pass

rmse,rmse_l,rmse_r = stereo_rig.calibrate([checkerboard._p3d]*len(corners_l),corners_l,corners_r)
print ("Calibrated:\n{}".format(stereo_rig))

stereo_rig.to_yaml("./calibration_data")

print ("Stored calibration at ./calibration_data")

print ("Rectifiying..")
print ("If calibration worked, corners should match chessboard in rectified image.")

def draw_epipolar_line(uv,img_source,img):
    max_depth = 1000
    min_depth = 0.001
    p_ccs_max = stereo_rig._left.image2camera(np.array(uv),max_depth)
    p_ccs_min = stereo_rig._left.image2camera(np.array(uv),min_depth)
    
    p_ccs_right_max = np.dot(stereo_rig._R,p_ccs_max) + stereo_rig._t
    p_ccs_right_min = np.dot(stereo_rig._R,p_ccs_min) + stereo_rig._t
    uv_right_max = stereo_rig._right.camera2image(p_ccs_right_max)
    uv_right_min = stereo_rig._right.camera2image(p_ccs_right_min)
    cv2.circle(img_source,uv.astype(int),2,(255,255,255))
    cv2.line(img,uv_right_min.reshape(2,).astype(int),uv_right_max.reshape(2,).astype(int),(255,255,255))


for img_left,img_right in load_images(args.source_left,args.source_right,image_resolution):
    
    img_calib_r = stereo_rig.rectify_right(img_right)
    img_calib_l = stereo_rig.rectify_left(img_left)
    try:
        img_left_rect = stereo_rig.rectify_right(img_left)
        img_right_rect = stereo_rig.rectify_left(img_right)
        corners_l_img  = find_corners(img_left_rect,"Left",checkerboard,True)
        corners_l_img = corners_l_img.reshape(-1,2)
       
        draw_epipolar_line(corners_l_img[0,:],img_left_rect,img_right_rect)
        draw_epipolar_line(corners_l_img[-1,:],img_left_rect,img_right_rect)
          
        #corners_r_img_f  = find_corners(img_right_rect,"Right",checkerboard)
        #corners_l_img_f  = find_corners(img_left_rect,"Left",checkerboard)
        #corners_r_img_f = np.reshape(corners_r_img_f,(-1,2))
        #corners_l_img_f = np.reshape(corners_l_img_f,(-1,2))

        #pixel_error_l = np.linalg.norm(corners_l_rect - corners_l_img_f)/len(corners_l_rect)
        #pixel_error_r = np.linalg.norm(corners_r_rect - corners_r_img_f)/len(corners_r_rect)
        
        #print ("Average Pixel Error: Left: {}, Right: {}".format(pixel_error_l,pixel_error_r))

        #cv2.drawChessboardCorners(img_left_rect, checkerboard._dimension, corners_l_rect, True)
        #cv2.drawChessboardCorners(img_right_rect, checkerboard._dimension, corners_r_rect, True)
        cv2.imshow('Left  Rect.', img_left_rect)
        cv2.imshow('Right Rect.', img_right_rect)
        cv2.waitKey(0)
    except CalibrationException:
        pass

