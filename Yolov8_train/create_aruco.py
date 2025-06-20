import cv2
import numpy as np

def cm_to_px(cm, dpi=300):
    return int((cm * dpi) / 2.54)

def create_marker(marker_id, cm_size, filename, dpi=300):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    px_size = cm_to_px(cm_size, dpi)

    marker = cv2.aruco.drawMarker(aruco_dict, marker_id, px_size)
    border = int(0.2 * px_size)
    marker_with_border = cv2.copyMakeBorder(marker, border, border, border, border, cv2.BORDER_CONSTANT, value=255)

    cv2.imwrite(filename, marker_with_border)
    print(f"Đã tạo mã ID={marker_id}, size={cm_size}cm, px={px_size}px → {filename}")

# Tạo 2 mã
create_marker(marker_id=1, cm_size=5.5, filename="aruco_5.5cm.png") # <-- MODIFY TAILLE
create_marker(marker_id=2, cm_size=8.0, filename="aruco_8cm.png")  # <-- MODIFY TAILLE