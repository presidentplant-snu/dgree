import cv2
import numpy as np
import time


def live_aruco_detection():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters()
    #detector = cv2.aruco.detectMarkers(aruco_dict, aruco_params)
    marker_size = 0.05   # 5cm

    #카메라 설정 & 초기화 대기
    cap = cv2.VideoCapture("./video.mp4")
    time.sleep(2)

    camera_matrix = np.eye(3, dtype=np.float32)


    while True: 
        ret, frame = cap.read()
        if not ret or frame is None or not isinstance(frame, np.ndarray):
            print("Failed to grab frame")
            break

        corners, ids, rejected = cv2.aruco.detectMarkers(frame,aruco_dict, parameters=aruco_params)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size,camera_matrix,dist_coeffs)

            for i in range(len(ids)):
                cv2.drawFrameAxes(frame)

                #marker의 3차원 좌표 표시
                pos_x = tvecs[i][0][0]
                pos_y = tvecs[i][0][1]
                pos_z = tvecs[i][0][2]

                #marker의 회전 정고 표시
                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]

                #marker의 중심 
                corner = corners[i][0]
                center_x = int(np.mean(corner[:,0]))
                center_y = int(np.mean(corner[:,1]))

                # marker 정보 화면 표시
                cv2.putText(frame, f"ID {ids[i][0]}",(center_x, center_y-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),2)
                cv2.putText(frame, f"Pos: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})m",(center_x, center_y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.putText(frame,f"Rot: ({euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f})deg",(center_x, center_y + 20),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2)

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
    live_aruco_detection()

if __name__== "__main__":
    main()


##dist_coeffs 등 camera calibration 정보 넘기기





