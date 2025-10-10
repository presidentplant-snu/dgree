import cv2
import numpy as np
import time


def live_aruco_detection(camera_matrix):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters_create()
    marker_size = 0.05   # 5cm

    #카메라 설정 & 초기화 대기
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError("카메라 연결 안됨")

    while True: 
        ret, frame = cap.read()
        if not ret or frame is None or not isinstance(frame, np.ndarray):
            print("Failed to grab frame")
            break

        cv2.imshow('ArUco Marker detection', frame.copy())
        corners, ids, rejected =  cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                #marker의 중심 
                corner = corners[i][0]

                #marker의 corner 표시
                for point in corner:
                    x, y = int(point[0]), int(point[1])
                    cv2.circle(frame, (x,y), 4, (0,0,255),-1)

        cv2.imshow('ArUco Marker detection', frame)

        if cv2.waitKey(1) & 0XFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()


def main(): 
    camera_matrix = [[1,0,0,0],[0,1,0,0],[0,0,1,0] ] 
    live_aruco_detection(camera_matrix)

if __name__== "__main__":
    main()


##dist_coeffs 등 camera calibration 정보 넘기기





