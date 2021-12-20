import cv2
import numpy as np

from globals import CalibrationException

def load_images(source_left:str,source_right,res):

    if source_left.startswith('/dev') or source_right.startswith('/dev'):
        print ("Live Mode..")
        #TODO replace this with opencv video capture from /dev
        from picamera import PiCamera
        cam_width = int((res[0]+31)/32)*32*2
        cam_height = int((res[1]+15)/16)*16

        # Initialize the camera
        capture = np.zeros((res[1], res[0]*2, 4), dtype=np.uint8)
        camera = PiCamera(stereo_mode='side-by-side', stereo_decimate=False)
        camera.resolution=(cam_width, cam_height)
        camera.framerate = 2
        #camera.vflip = True
        print ("Streaming..")
        i = 0
        for f in camera.capture_continuous(capture, format="bgra",use_video_port=True, resize=(cam_width,cam_height)):
            i += 1
            f = cv2.cvtColor(f,cv2.COLOR_BGR2GRAY)
            img_right = f[:,0:int(f.shape[1]/2)] #Y+H and X+W
            img_left = f[:,int(f.shape[1]/2):f.shape[1]]
            print ("Frame: %d" % i)

            yield img_left,img_right
    else:
        print("Collecting images..")
        import glob
        files_left = glob.glob(source_left)
        files_right = glob.glob(source_right)

        files_left.sort()
        files_right.sort()
        n_samples = min(len(files_left),len(files_right))
        for i in range(n_samples):
            f_left = files_left[i]
            f_right = files_right[i]
            yield cv2.resize(cv2.imread(f_left,cv2.IMREAD_GRAYSCALE),res), cv2.resize(cv2.imread(f_right,cv2.IMREAD_GRAYSCALE),res)


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
    print ( "Chessboard found!")
    minx = corners[:,:,0].min()
    maxx = corners[:,:,0].max()
    miny = corners[:,:,1].min()
    maxy = corners[:,:,1].max()

    """
    Prevent subsequent failure if chessboard is too close to the side. 
    Thanks to: https://github.com/realizator/stereopi-fisheye-robot/blob/master/4_calibration_fisheye.py
    """

    if minx<img.shape[1]/7 or miny < img.shape[0]/7:
        raise CalibrationException("Chessboard too close to side!")

    cv2.cornerSubPix(img,corners,(3,3),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
    if show:
        img_cp = img.copy()
        cv2.drawChessboardCorners(img_cp, checkerboard._dimension, corners, ret)
        cv2.imshow(name, img_cp)
        key = cv2.waitKey(10)

    return corners
