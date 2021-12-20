import cv2
import numpy as np

from settings import CalibrationException

def load_images(source:str,res):

    if source == "live":
        from picamera import PiCamera
        cam_width = int((res[0]+31)/32)*32
        cam_height = int((res[1]+15)/16)*16

        # Initialize the camera
        capture = np.zeros((res[1], res[0], 4), dtype=np.uint8)
        camera = PiCamera(stereo_mode='side-by-side', stereo_decimate=False)
        camera.resolution=(cam_width, cam_height)
        camera.framerate = 2
        camera.vflip = True
        for f in camera.capture_continuous(capture, format="bgra",use_video_port=True, resize=res):
            gray = cv2.cvtColor(f,cv2.COLOR_BGR2GRAY)
            img_left = gray[:,0:int(gray.shape[1]/2)] #Y+H and X+W
            img_right = gray[:,int(gray.shape[1]/2):gray.shape[1]]
            print("Shape Total: {} R: {} L: {}".format(gray.shape,img_left.shape,img_right.shape))
            yield img_left,img_right
    else:
        import glob
        files = glob.glob(source + '/left_*.png')
        files.sort()
        for f in files:
            fright = f.replace('left','right')
            print ("Loading: {}, {}".format(f,fright))
            yield cv2.imread(f,cv2.IMREAD_GRAYSCALE),cv2.imread(fright,cv2.IMREAD_GRAYSCALE)


class Checkerboard:

    def __init__(self,dimension,square_size):
        self._dimension = dimension
        self._square_size = square_size
        self._p3d = np.zeros( (dimension[0]*dimension[1], 1, 3) , np.float64)
        self._p3d[:,0, :2] = np.mgrid[0:dimension[0], 0:dimension[1]].T.reshape(-1, 2)
        self._p3d *= square_size

def find_corners(img,name,checkerboard:Checkerboard,show=True):
    ret, corners = cv2.findChessboardCorners(img, checkerboard._dimension, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    if ret == False:
        raise CalibrationException("No chessboard found!")
    minx = corners[:,:,0].min()
    maxx = corners[:,:,0].max()
    miny = corners[:,:,1].min()
    maxy = corners[:,:,1].max()

    border_threshold_x = img.shape[1]/7
    border_threshold_y = img.shape[0]/7
    print ("thr_X: ", border_threshold_x, "thr_Y:", border_threshold_y)

    x_thresh_bad = False
    if ((minx<border_threshold_x)): # or (loadedX-maxRx < border_threshold_x) or (loadedX-maxLx < border_threshold_x)):
        x_thresh_bad = True
    y_thresh_bad = False
    if ((miny<border_threshold_y) ): # or (loadedY-maxRy < border_threshold_y) or (loadedY-maxLy < border_threshold_y)):
        y_thresh_bad = True
    if (y_thresh_bad==True) or (x_thresh_bad==True):
        ret = False
        raise CalibrationException("Chessboard too close to side!")

    cv2.cornerSubPix(img,corners,(3,3),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
    if show:
        cv2.drawChessboardCorners(img, checkerboard._dimension, corners, ret)
        cv2.imshow(name, img)
        key = cv2.waitKey(10)

    return corners
