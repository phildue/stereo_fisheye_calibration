import cv2

assert cv2.__version__[0] == '4', 'The fisheye module requires opencv version >= 3.0.0'
from typing import List
import numpy as np

from settings import CalibrationException

class Camera:
    def __init__(self,name:str, resolution:tuple, K = np.eye(3), d = np.zeros((4,1)), R = np.eye(3), P = np.zeros((3,4))):
        self._K = K
        self._d = d
        self._R = R
        self._P = P
        self._dist_model = "equidistant"
        self._name = name
        self._width = resolution[0]
        self._height = resolution[1]
        self._resolution = resolution

    def calibrate(self,objpoints,imgpoints):
        assert len(objpoints) == len(imgpoints)
        N_OK = len(objpoints)
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
        try:
            rms, self._K,self._d, _, _ = cv2.fisheye.calibrate(
                objpoints,
                imgpoints,
                self._resolution,
                None,
                None,
                rvecs,
                tvecs,
                calibration_flags,
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
        except Exception as e:
            raise CalibrationException(str(e))
        
        self._lutX, self._lutY = cv2.fisheye.initUndistortRectifyMap(self._K, self._d, np.eye(3),self._K, self._resolution, cv2.CV_16SC2)
        return rms
    
    def rectify(self,img:np.array) -> np.array:
        return cv2.remap(img, self._lutX, self._lutY, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)

    def to_yaml(self) -> str:
        def format_mat(x, precision):
            return ("[%s]" % (
                np.array2string(x, precision=precision, suppress_small=True, separator=", ")
                    .replace("[", "").replace("]", "").replace("\n", "\n        ")
            ))

        assert self._K.shape == (3, 3)
        assert self._R.shape == (3, 3)
        assert self._P.shape == (3, 4)
        yml = "\n".join([
            "image_width: %d" % self._width,
            "image_height: %d" % self._height,
            "camera_name: " + self._name,
            "camera_matrix:",
            "  rows: 3",
            "  cols: 3",
            "  data: " + format_mat(self._K, 5),
            "distortion_model: " + self._dist_model,
            "distortion_coefficients:",
            "  rows: 1",
            "  cols: %d" % self._d.size,
            "  data: [%s]" % ", ".join("%8f" % x for x in self._d.flat),
            "rectification_matrix:",
            "  rows: 3",
            "  cols: 3",
            "  data: " + format_mat(self._R, 8),
            "projection_matrix:",
            "  rows: 3",
            "  cols: 4",
            "  data: " + format_mat(self._P, 5),
            ""
        ])
        return yml

class StereoCam:
    def __init__(self,name, resolution:tuple, R=np.eye(3),t=np.zeros((3,1)),Q=np.zeros((4,4))):
        self._R = R
        self._t = t
        self._left = Camera(name+"_left", resolution)
        self._right = Camera(name+"_right", resolution)
        self._name = name
        self._resolution = resolution
        self._Q = Q

    def calibrate(self,obj_wcs:List[np.array], corners_l:List[np.array], corners_r:List[np.array]):
        assert len(obj_wcs) == len(corners_l) == len(corners_r)
     
        rms_l = self._left.calibrate(obj_wcs,corners_l)
        rms_r = self._right.calibrate(obj_wcs,corners_r)
            # We need a lot of variables to calibrate the stereo camera
        """
        Based on code from:
        https://gist.github.com/aarmea/629e59ac7b640a60340145809b1c9013
        """
        processing_time01 = cv2.getTickCount()

        OPTIMIZE_ALPHA = 0.25

        print("Calibrating cameras together...")

        pixelsLeft = np.asarray(corners_l, dtype=np.float64)
        pixelsRight = np.asarray(corners_r, dtype=np.float64)
        TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        try:
            # Stereo calibration
            (rms, _, _, self.R, self.t, _, _) = cv2.fisheye.stereoCalibrate(
                    obj_wcs, pixelsLeft, pixelsRight,
                    self._left._K, self._left._d,
                    self._right._K, self._right._d,
                    self._resolution, None, None,
                    cv2.CALIB_FIX_INTRINSIC, TERMINATION_CRITERIA)
        except Exception as e:
            raise CalibrationException(str(e))
       
        # Rectify calibration results
        self._left._R, self._right._R, self._left._P, self._right._P, self._Q = cv2.fisheye.stereoRectify(
                        self._left._K, self._left._d,
                        self._right._K, self._right._d,
                        self._resolution, self._R, self._t,
                        cv2.CALIB_ZERO_DISPARITY,
                        None, None, None, None, None,
                        (0,0) , 0, 0)
        self._left._P[:3,:3] = np.dot(self._left._K,self._left._R)
        self._right._P[:3,:3] = np.dot(self._right._K,self._right._R)

        self._left._lutX, self._left._lutY = cv2.fisheye.initUndistortRectifyMap(
        self._left._K, self._left._d, self._left._R,
        self._left._P, self._resolution, cv2.CV_16SC2)

        self._right._lutX, self._right._lutY = cv2.fisheye.initUndistortRectifyMap(
        self._right._K, self._right._d, self._right._R,
        self._right._P, self._resolution, cv2.CV_16SC2)
        
        return rms, rms_l, rms_r
    
    def rectify_left(self,img:np.array) -> np.array:
        return self._left.rectify(img)
    
    def rectify_right(self,img:np.array) -> np.array:
        return self._right.rectify(img)


