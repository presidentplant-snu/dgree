import cv2
import numpy as np
import math
import time
import pickle

def aruco_detection(dist_coeffs, camera_matrix):
    
    # ArUco 검출기 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters()
    #detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    # 마커 크기 설정 
    marker_size = 0.05  # 0.05m
    
    # 카메라 설정
    cap = cv2.VideoCapture(0)
    
    # 카메라 초기화 대기
    time.sleep(0.5)
    
    marker_infos= {}

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
            
        # 이미지 왜곡 보정
        frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        
        # 마커 검출
        corners, ids, _ = cv2.aruco.detectMarkers(frame_undistorted, aruco_dict, parameters = aruco_params)
        
        # 마커가 검출되면 표시 및 포즈 추정
        if ids is not None:
            # 검출된 마커 표시
            cv2.aruco.drawDetectedMarkers(frame_undistorted, corners, ids)
            
            # 각 마커의 포즈 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, camera_matrix, dist_coeffs
            )
            
            # 각 마커에 대해 처리
            for i in range(len(ids)):

                # 좌표축 표시
                cv2.drawFrameAxes(frame_undistorted, camera_matrix, dist_coeffs, 
                                rvecs[i], tvecs[i], marker_size/2)
                
                # 마커의 3D 위치 표시
                pos_x = tvecs[i][0][0]
                pos_y = tvecs[i][0][1]
                pos_z = tvecs[i][0][2]
                
                # 회전 벡터를 오일러 각도로 변환
                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
                
                # 마커 정보 표시
                corner = corners[i][0]
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                
                marker_infos[i] = {
                    'pos': tvecs.astype(float),     # x,y,z [m]
                    'euler': euler_angles.astype(float)
                }

                '''
                cv2.putText(frame_undistorted, 
                          f"ID: {ids[i][0]}", 
                          (center_x, center_y - 40), 
                          cv2.FONT_HERSHEY_SIMPLEX, 
                          0.5, (0, 0, 0), 2)
                          
                cv2.putText(frame_undistorted,
                          f"Pos: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})m",
                          (center_x, center_y),
                          cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (0, 0, 0), 2)
                          
                cv2.putText(frame_undistorted,
                          f"Rot: ({euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f})deg",
                          (center_x, center_y + 20),
                          cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (0, 0, 0), 2)
                
                # 코너 포인트 표시
                for point in corner:
                    x, y = int(point[0]), int(point[1])
                    cv2.circle(frame_undistorted, (x, y), 4, (0, 0, 255), -1)
                
        # 프레임 표시
        cv2.imshow('ArUco Marker Detection', frame_undistorted)
        '''
        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 리소스 해제
    cap.release()
    cv2.destroyAllWindows()

    return marker_infos


def angle_btw_markers(marker_infos):

    p0 = marker_infos[0]['pos'].reshape(-1) 
    p1 = marker_infos[1]['pos'].reshape(-1)
    p2 = marker_infos[2]['pos'].reshape(-1)

    v1 = p0-p1
    v2 = p2-p1
    
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    
    cos_ang= np.clip(np.dot(v1,v2)/(n1*n2), -1.0,1.0)
    ang = math.degrees(math.acos(cos_ang))

    return ang




def main():


    f = cv2.FileStorage("camera_params.yml", cv2.FILE_STORAGE_READ)
    if not f.isOpened():
        FileNotFoundError(f"Cannot open")

    dist_coeffs = f.getNode("dist_coeffs").mat()
    camera_matrix = f.getNode("camera_matrix").mat()

    f.release()

    print("Starting ArUco marker detection...")

    ids = [1, 2, 3]
    marker_infos = aruco_detection(dist_coeffs, camera_matrix)

    joint_coordinates = np.stack([marker_infos[i]['pos'] for i in ids], axis=0)
    current_angles = angle_btw_markers(marker_infos)

if __name__ == "__main__":
    main()