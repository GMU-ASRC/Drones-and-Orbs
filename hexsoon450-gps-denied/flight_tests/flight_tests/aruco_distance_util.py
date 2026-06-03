import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import os
import argparse

class ArucoSingleTracker():
    def __init__(self,
                id_to_find,
                marker_size,
                camera_matrix,
                camera_distortion,
                camera_size=[640,480]):
        
        self.id_to_find     = id_to_find
        self.marker_size    = marker_size
        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion
        self.is_detected    = False
        self._kill          = False
        
        self._R_flip      = np.zeros((3,3), dtype=np.float32)
        self._R_flip[0,0] = 1.0
        self._R_flip[1,1] =-1.0
        self._R_flip[2,2] =-1.0

        self._aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self._parameters  = aruco.DetectorParameters_create()

        self._cap = cv2.VideoCapture(0)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])

        self._t_read      = time.time()
        self._t_detect    = self._t_read
        self.fps_read    = 0.0
        self.fps_detect  = 0.0    

    def _rotationMatrixToEulerAngles(self,R):
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6        
        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def _update_fps_read(self):
        t           = time.time()
        self.fps_read    = 1.0/(t - self._t_read)
        self._t_read      = t
        
    def _update_fps_detect(self):
        t           = time.time()
        self.fps_detect  = 1.0/(t - self._t_detect)
        self._t_detect      = t    

    def stop(self):
        self._kill = True

    def track(self, loop=True):
        self._kill = False
        marker_found = False
        x = y = z = 0
        
        while not self._kill:
            ret, frame = self._cap.read()
            if not ret:
                break
                
            self._update_fps_read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict, 
                            parameters=self._parameters,
                            cameraMatrix=self._camera_matrix, 
                            distCoeff=self._camera_distortion)
                            
            if not ids is None and self.id_to_find in ids[0]:
                marker_found = True
                self._update_fps_detect()
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix, self._camera_distortion)

                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                x = tvec[0]
                y = tvec[1]
                z = tvec[2]

                R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc    = R_ct.T

                roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)
                pos_camera = -R_tc*np.matrix(tvec).T
                roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)
            
            if not loop: return(marker_found, x, y, z)


def example_distance_measure_script():
    vs = cv2.VideoCapture(0)
    time.sleep(2.0)

    arucoParams = cv2.aruco.DetectorParameters_create()
    arucoParams_2 = cv2.aruco.DetectorParameters_create()
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    arucoDict_2 = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)

    CACHED_PTS = None
    CACHED_IDS = None
    Line_Pts = None
    measure = None

    CACHED_PTS_2 = None
    CACHED_IDS_2 = None
    Line_Pts_2 = None
    measure_2 = None
    
    while True:
        Dist = []
        ret, image = vs.read()
        if not ret:
            break
            
        h, w = image.shape[:2]
        image = cv2.resize(image, (800, int(h * 800 / w)))
        corners, ids, rejected= cv2.aruco.detectMarkers( image, arucoDict, parameters=arucoParams)

        if len(corners) <= 0 and CACHED_PTS is not None:
            corners = CACHED_PTS
            
        if len(corners) > 0:
            CACHED_PTS = corners
            if ids is not None:
                ids = ids.flatten()
                CACHED_IDS = ids
            elif CACHED_IDS is not None:
                ids = CACHED_IDS
                
            if len(corners) < 2 and len(CACHED_PTS) >= 2:
                corners = CACHED_PTS
                
            for (markerCorner, markerId) in zip(corners, ids):
                corners_abcd = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_abcd
                cX = int((topLeft[0] + bottomRight[0])//2)
                cY = int((topLeft[1] + bottomRight[1])//2)

                measure = abs(3.5 / (topLeft[0] - cX))
                Dist.append((cX, cY))
                
                if len(Dist) == 0 and Line_Pts is not None:
                    Dist = Line_Pts
                if len(Dist) == 2:
                    Line_Pts = Dist
                    ed = ((Dist[0][0] - Dist[1][0]) ** 2 + ((Dist[0][1] - Dist[1][1]) ** 2)) ** (0.5)

        Dist_2 = []
        corners_2, ids_2, rejected_2 = cv2.aruco.detectMarkers(image, arucoDict_2, parameters=arucoParams_2)
        if len(corners_2) <= 0 and CACHED_PTS_2 is not None:
            corners_2 = CACHED_PTS_2
            
        if len(corners_2) > 0:
            CACHED_PTS_2 = corners_2
            if ids_2 is not None:
                ids_2 = ids_2.flatten()
                CACHED_IDS_2 = ids_2
            elif CACHED_IDS_2 is not None:
                ids_2 = CACHED_IDS_2
                
            if len(corners_2) < 2 and len(CACHED_PTS_2) >= 2:
                corners_2 = CACHED_PTS_2
                
            for (markerCorner_2, markerId_2) in zip(corners_2, ids_2):
                corners_abcd_2 = markerCorner_2.reshape((4, 2))
                (topLeft_2, topRight_2, bottomRight_2, bottomLeft_2) = corners_abcd_2
                cX_2 = int((topLeft_2[0] + bottomRight_2[0]) // 2)
                cY_2 = int((topLeft_2[1] + bottomRight_2[1]) // 2)

                measure_2 = abs(3.5 / (topLeft_2[0] - cX_2))
                Dist_2.append((cX_2, cY_2))
                
                if len(Dist_2) == 0 and Line_Pts_2 is not None:
                    Dist_2 = Line_Pts_2
                if len(Dist_2) == 2:
                    Line_Pts_2 = Dist_2
                    ed_2 = ((Dist_2[0][0] - Dist_2[1][0]) ** 2 + ((Dist_2[0][1] - Dist_2[1][1]) ** 2)) ** (0.5)

    vs.release()

def get_m_per_px(corners_abcd, marker_size_m):
    topLeft = corners_abcd[0]
    bottomRight = corners_abcd[2]
    cX = (topLeft[0] + bottomRight[0]) / 2.0
    
    half_width_px = abs(topLeft[0] - cX)
    if half_width_px < 1.0:
        return 0.001

    return (marker_size_m / 2.0) / half_width_px

if __name__ == "__main__":
    calib_path  = ""
    camera_matrix = np.eye(3)
    camera_distortion = np.zeros(5)

    try:
        camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
        camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')  
    except Exception:
        pass

    aruco_tracker = ArucoSingleTracker(id_to_find=72, marker_size=10, camera_matrix=camera_matrix, camera_distortion=camera_distortion)
