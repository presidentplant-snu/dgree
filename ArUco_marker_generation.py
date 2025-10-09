import cv2
import numpy as np
import os
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)


marker_ids = range(3)
marker_size = 100
output_dir = "marker_imgs"

for marker_id in marker_ids:
    output_filename = f"joint_{marker_id}.png"
    output_path = os.path.join(output_dir, output_filename)
    marker_img = np.zeros((marker_size, marker_size, 1),dtype = "uint8")
    cv2.aruco.drawMarker(dictionary, marker_id, marker_size, marker_img,1)
    cv2.imwrite(output_path, marker_img)
    cv2.imshow("joint_{marker_id}", marker_img)

cv2.waitKey(0)
cv2.destroyAllWindows()