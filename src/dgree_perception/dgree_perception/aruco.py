import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2
import math

# joint 위치
# current angle



do_imshow = True


class arucoDetectionNode(Node):
    def __init__(self, camera_matrix):
        super().__init__('marker_detector_node')
        
        self.camera_matrix = camera_matrix
        self.detection = self.live_aruco_detection
        self.marker_infos = np.ndarray((4,2))
        self.pos_pub = self.create_publisher(Float32MultiArray,'/markers/joint_states',10)
        self.ang_pub = self.create_publisher(Float32MultiArray, '/markers/current_angles',10)
        self.ang = [0,0]
        self.timer = self.create_timer(1.0/30.0, self.control_loop)

        #카메라 설정 & 초기화 대기
        self.cap = cv2.VideoCapture(0)


    def control_loop(self):
        self.live_aruco_detection()

        pos_info = self.marker_infos.flatten()  
        marker_pos = Float32MultiArray()
        marker_pos.data = pos_info.flatten()

        marker_ang = Float32MultiArray()
        marker_ang.data= self.ang

        self.pos_pub.publish(marker_pos)
        self.ang_pub.publish(marker_ang)


    def live_aruco_detection(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        aruco_params = cv2.aruco.DetectorParameters_create()
        marker_size = 0.05   # 5cm

       
        if not self.cap.isOpened():
            raise RuntimeError("카메라 연결 안됨")

         
        ret, frame = self.cap.read()
        if not ret or frame is None or not isinstance(frame, np.ndarray):
            print("Failed to grab frame")
            return 

        cv2.imshow('ArUco Marker detection', frame.copy())
        corners, ids, rejected =  cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                #marker의 
                # 중심 
                sum = np.array([0.0,0.0])

                for corner in corners[i]:
                    
                    if do_imshow:
                        for point in corner:
                            sum += point
                            x, y = int(point[0]), int(point[1])
                            cv2.circle(frame, (x,y), 4, (0,0,255),-1)

                center = sum/4
                if ids[i] > 3:
                    continue
                self.marker_infos[ids[i],0] = center[0]
                self.marker_infos[ids[i],1] = center[1]
                
        if do_imshow:
            cv2.imshow('ArUco Marker detection', frame)
            
                    
        if cv2.waitKey(1) & 0XFF == ord('q'):
            return 
    
        self.angle_btw_markers()
    
    def angle_btw_markers(self):
        p0 = self.marker_infos[0]
        p1 = self.marker_infos[1]
        p2 = self.marker_infos[2]
        p3 = self.marker_infos[3]


        v1 = p1-p0
        v2 = p2-p1
        v3 = p3-p2

        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        n3 = np.linalg.norm(v3)

        ang1 = np.arctan2(v1[0],v1[1])
        ang2 = np.arctan2(v2[0],v2[1])
        ang3 = np.arctan2(v3[0],v3[1])

        '''
        cos_ang1= np.clip(np.dot(v1,v2)/(n1*n2), -1.0,1.0)
        ang1 = math.degrees(math.acos(cos_ang1))

        cos_ang2 = np.clip(np.dot(v2,v3)/(n2*n3), -1.0,1.0)
        ang2 = math.degrees(math.acos(cos_ang2))

        '''
        def normalize(angle):
            # Numpy version is faster
            return np.arctan2(np.sin(angle),np.cos(angle))
        self.ang = np.array([normalize(ang2-ang1), normalize(ang3-ang2)])
        


def main(args = None):
    rclpy.init(args=args)
    camera_matrix = [[1,0,0,0],[0,1,0,0],[0,0,1,0] ] 
    node = arucoDetectionNode(camera_matrix)

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

