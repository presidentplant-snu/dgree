import cv2
import numpy as np
from datetime import datetime


# ===== 사용자 설정 =====
# 체커보드 내부 코너 (가로, 세로)
CHESSBOARD = (10, 7)
# 한 칸의 한 변 길이
SQUARE_SIZE = 25.0
# 샘플 프레임 개수
MIN_SAMPLES = 12
# 파일 저장 경로
SAVE_PATH = "camera_params.yml"

def save_params(path, dist_coeffs, camera_matrix):

    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("camera_matrix", camera_matrix)
    fs.release()

def main():
    # 체커보드 3D 기준점 (Z=0 평면)
    objp = np.zeros((CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = []  # 3D 포인트
    imgpoints = []  # 2D 포인트

    cap = cv2.VideoCapture(0)  
    if not cap.isOpened():
        raise RuntimeError("카메라 연결 안됨")

    print("현재 프레임을 샘플로 채택 : c, 캘리브레이션 진행/종료: q")
    sample_count = 0
    last_found = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD,
                                                   flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, CHESSBOARD, corners, found)
            last_found = True
        else:
            last_found = False

        msg1 = f"Samples: {sample_count}/{MIN_SAMPLES} | 'c': capture  'q': calibrate/quit"
        cv2.putText(display, msg1, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        if found:
            cv2.putText(display, "Chessboard detected", (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(display, "Show a clear chessboard to the camera", (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (60, 60, 255), 2, cv2.LINE_AA)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            if last_found:
                # 코너 정밀화
                corners_sub = cv2.cornerSubPix(
                    gray, corners, winSize=(11, 11), zeroZone=(-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
                objpoints.append(objp.copy())
                imgpoints.append(corners_sub)
                sample_count += 1
                print(f"샘플 채택 완료: {sample_count}")
            else:
                print("체커보드를 먼저 정확히 인식시켜 주세요.")

        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(objpoints) < 3:
        print("샘플이 부족합니다. 최소 3장 이상 필요합니다(권장 10장 이상).")
        return

    # 카메라 캘리브레이션
    img_size = (gray.shape[1], gray.shape[0])  # (width, height)
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objectPoints=objpoints,
        imagePoints=imgpoints,
        imageSize=img_size,
        cameraMatrix=None,
        distCoeffs=None
    )

    print("=== Calibration Result ===")
    print("RMS reprojection error:", rms)
    print("camera_matrix:\n", camera_matrix)
    print("dist_coeffs:\n", dist_coeffs.ravel())

    save_params(SAVE_PATH, dist_coeffs, camera_matrix, image_size=img_size, rms=rms)

if __name__ == "__main__":
    main()
