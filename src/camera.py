import cv2

assert cv2.__version__[0] == '4', 'The fisheye module requires opencv version >= 3.0.0'
from typing import List
import numpy as np
import yaml

from globals import CalibrationException

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
        self._rmse = 0

    def fx(self):
        return self._K[0,0]
    def fy(self):
        return self._K[1,1]
    def cx(self):
        return self._K[0,2]
    def cy(self):
        return self._K[1,2]

    def calibrate(self,objpoints,imgpoints):
        assert len(objpoints) == len(imgpoints)
        N_OK = len(objpoints)
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        #calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND
        try:
            self._rmse, self._K,self._d, _, _ = cv2.fisheye.calibrate(
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
        return self._rmse
    
    def rectify(self,img:np.array) -> np.array:
        return cv2.remap(img, self._lutX, self._lutY, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)

    def rectify_points(self,points:np.array):
        points_u_normalized =  cv2.fisheye.undistortPoints(points, self._K, self._d)
        points_u_normalized = points_u_normalized.reshape(-1,2)

        points_u = np.zeros_like(points_u_normalized)

        for i, (x, y) in enumerate(points_u_normalized):
            points_u[i,0] = x*self.fx() + self.cx()
            points_u[i,1] = y*self.fy() + self.cy()

        return points_u


    def to_yaml(self, filename:str) -> str:
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

        with open(filename, 'w') as f:
            f.write(yml)
    def from_yaml(self,filename:str):
        with open(filename) as f:
            read_data = yaml.load(f, Loader=yaml.FullLoader)
            self._K = np.array(read_data['camera_matrix']['data']).reshape(3,3)
            self._d = np.array(read_data['distortion_coefficients']['data']).reshape(int(read_data['distortion_coefficients']['rows']),int(read_data['distortion_coefficients']['cols']))
            self._R = np.array(read_data['rectification_matrix']['data']).reshape(3,3)
            self._P = np.array(read_data['projection_matrix']['data']).reshape(3,4)
            self._name = read_data['camera_name']
        self._lutX, self._lutY = cv2.fisheye.initUndistortRectifyMap(self._K, self._d, np.eye(3),self._K, self._resolution, cv2.CV_16SC2)

    def __repr__(self) -> str:
        return "Name:{}\nK=\n{}\nd=\n{}\nR=\n{}\nP=\n{}\nCalibration Error (RMSE): {:0.3f}".format(self._name,self._K,self._d,self._R,self._P,self._rmse)
class StereoCam:
    def __init__(self,name, resolution:tuple, R=np.eye(3),t=np.zeros((3,1)),Q=np.zeros((4,4))):
        self._R = R
        self._t = t
        self._left = Camera(name+"_left", resolution)
        self._right = Camera(name+"_right", resolution)
        self._name = name
        self._resolution = resolution
        self._width = resolution[0]
        self._height = resolution[1]
        self._Q = Q
        self._rmse_l = 0
        self._rmse_r = 0
        self._rmse = 0


    def calibrate(self,obj_wcs:List[np.array], corners_l:List[np.array], corners_r:List[np.array]):
        assert len(obj_wcs) == len(corners_l) == len(corners_r)
     
        self._left.calibrate(obj_wcs,corners_l)
        self._right.calibrate(obj_wcs,corners_r)

        """
        Based on code from:
        https://gist.github.com/aarmea/629e59ac7b640a60340145809b1c9013
        """
        processing_time01 = cv2.getTickCount()

        OPTIMIZE_ALPHA = 0.25

        pixelsLeft = np.asarray(corners_l, dtype=np.float64)
        pixelsRight = np.asarray(corners_r, dtype=np.float64)
        objPoints = np.asarray(obj_wcs, dtype=np.float64)

        TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        try:
            (self._rmse, _,_,_,_ , _, _) = cv2.fisheye.stereoCalibrate(
                    objPoints, pixelsLeft, pixelsRight,
                    self._left._K, self._left._d,
                    self._right._K, self._right._d,
                    self._resolution, self._R,  self._t,
                    cv2.CALIB_FIX_INTRINSIC, TERMINATION_CRITERIA)
        except Exception as e:
            raise CalibrationException(str(e))

        try:
            self._left._R, self._right._R, self._left._P, self._right._P, self._Q = cv2.fisheye.stereoRectify(
                            self._left._K, self._left._d,
                            self._right._K, self._right._d,
                            self._resolution, self._R, self._t,
                            cv2.CALIB_ZERO_DISPARITY,
                            None, None, None, None, None,
                            (0,0) , 0, 0)
        except Exception as e:
            raise CalibrationException(str(e))
        

        # Taken from ros2: https://github.com/ros-perception/image_pipeline/blob/8040dc8af10e04292ac181b2e9ab1edecad3758c/camera_calibration/src/camera_calibration/calibrator.py#L1216
        # Seems computation of P goes wrong in stereo rectify we compute it manually here
        self._left._P[:3,:3] = np.dot(self._left._K,self._left._R)
        self._right._P[:3,:3] = np.dot(self._right._K,self._right._R)

        self._left._lutX, self._left._lutY = cv2.fisheye.initUndistortRectifyMap(
        self._left._K, self._left._d, self._left._R,
        self._left._P, self._resolution, cv2.CV_16SC2)

        self._right._lutX, self._right._lutY = cv2.fisheye.initUndistortRectifyMap(
        self._right._K, self._right._d, self._right._R,
        self._right._P, self._resolution, cv2.CV_16SC2)
        
        return self._rmse, self._left._rmse, self._right._rmse
    
    def rectify_left(self,img:np.array) -> np.array:
        return self._left.rectify(img)
    
    def rectify_right(self,img:np.array) -> np.array:
        return self._right.rectify(img)

    def rectify_points_left(self,points:np.array) -> np.array:
        return self._left.rectify_points(points)
    
    def rectify_points_right(self,points:np.array) -> np.array:
        return self._right.rectify_points(points)


    def __repr__(self) -> str:
        return "Left:\n{}"\
        "\nRight:\n{}"\
        "\nR=\n{}\nt={}"\
        "\nQ=\n{}"\
        "\nCalibration Error (RMSE): {:0.3f}".format(self._left,self._right,self._R,self._t.transpose(),self._Q,self._rmse)
    
    def to_yaml(self,folder:str):
        import os
        if not os.path.exists(folder):
            os.mkdir(folder)
        file_left = folder + "/left.yaml"
        file_right = folder + "/right.yaml"
        self._left.to_yaml(file_left)
        self._right.to_yaml(file_right)

        def format_mat(x, precision):
            return ("[%s]" % (
                np.array2string(x, precision=precision, suppress_small=True, separator=", ")
                    .replace("[", "").replace("]", "").replace("\n", "\n        ")
            ))

        assert self._Q.shape == (4, 4)
        assert self._R.shape == (3, 3)
        assert self._t.shape == (3, 1)
        yml = "\n".join([
            "left: left.yaml",
            "right: right.yaml",
            "image_width: %d" % self._width,
            "image_height: %d" % self._height,
            "camera_name: " + self._name,
            "disparity_to_depth_matrix:",
            "  rows: 4",
            "  cols: 4",
            "  data: " + format_mat(self._Q, 5),
            "rotation_matrix:",
            "  rows: 3",
            "  cols: 3",
            "  data: " + format_mat(self._R, 8),
            "translation_vector:",
            "  rows: 3",
            "  cols: 1",
            "  data: " + format_mat(self._t, 5),
            ""
        ])
        with open(folder+"/stereo.yaml", 'w') as f:
            f.write(yml)

    def from_yaml(self,foldername:str):
        with open(foldername + "/stereo.yaml") as f:
            read_data = yaml.load(f, Loader=yaml.FullLoader)
            self._left.from_yaml(foldername + "/" + read_data['left'])
            self._right.from_yaml(foldername + "/" + read_data['right'])
            self._Q = np.array(read_data['disparity_to_depth_matrix']['data']).reshape(4,4)
            self._R = np.array(read_data['rotation_matrix']['data']).reshape(3,3)
            self._t = np.array(read_data['translation_vector']['data']).reshape(3,1)
            self._name = read_data['camera_name']
      
        self._left._lutX, self._left._lutY = cv2.fisheye.initUndistortRectifyMap(
        self._left._K, self._left._d, self._left._R,
        self._left._P, self._resolution, cv2.CV_16SC2)

        self._right._lutX, self._right._lutY = cv2.fisheye.initUndistortRectifyMap(
        self._right._K, self._right._d, self._right._R,
        self._right._P, self._resolution, cv2.CV_16SC2)
        
