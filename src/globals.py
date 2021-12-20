import cv2
import numpy as np

# Global variables preset
max_frames = 50

# Image resolution for processing
img_width = 640
img_height = 480
image_resolution = (img_width,img_height)

checkerboard = (6,9)
square_size = 0.025

class CalibrationException(Exception):
    pass