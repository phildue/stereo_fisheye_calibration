  
from camera import StereoCam
from image_collection import load_images,Checkerboard,find_corners
from globals import CalibrationException,checkerboard,square_size,image_resolution,max_frames
import argparse

import os
import cv2
import numpy as np



checkerboard = Checkerboard(checkerboard,square_size)

parser = argparse.ArgumentParser()
parser.add_argument('--source',
                    help='First argument',
                    default="calibration_images",
                    )
parser.add_argument('--model',
                    help='[Fisheye|Pinhole]',
                    default="fisheye",
                    )
args = parser.parse_args()

corners_l = [] # 2d points in image plane.
corners_r = [] # 2d points in image plane.

fNo = 0
for grayR,grayL in load_images("./calibration_images",image_resolution):
    loadedY, loadedX  =  grayL.shape

    try:
        corners_r_img  = find_corners(grayR,"Right",checkerboard)
        corners_l_img  = find_corners(grayL,"Left",checkerboard)

        corners_l.append(corners_l_img)
        corners_r.append(corners_r_img)
        fNo += 1
        if fNo > max_frames:
            break
    except CalibrationException:
        pass

stereo_rig = StereoCam("stereopi",image_resolution)
rmse,rmse_l,rmse_r = stereo_rig.calibrate([checkerboard._p3d]*len(corners_l),corners_l,corners_r)

for img_right,img_left in load_images(args.source,image_resolution):
    
    img_calib_r = stereo_rig.rectify_right(img_right)
    img_calib_l = stereo_rig.rectify_left(img_left)

    cv2.imshow('Left  STEREO CALIBRATED', img_calib_l)
    cv2.imshow('Right STEREO CALIBRATED', img_calib_r)
    cv2.waitKey(100)

cv2.waitKey(0)

left_yaml = stereo_rig._left.to_yaml()
right_yaml = stereo_rig._right.to_yaml()
print ("Calibration done! \n Left : \n{} \n Right: \n{} \nRMSE: {} l: {} r: {}".format(left_yaml,right_yaml,rmse,rmse_l,rmse_r))

with open('left.yaml', 'w') as f:
    f.write(left_yaml)

with open('right.yaml', 'w') as f:
    f.write(right_yaml)