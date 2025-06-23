import cv2
import depthai as dai
import numpy as np
from ultralytics import YOLO

# ==== Camera intrinsics (adjust with calibration if needed) ====
fx, fy = 514.28, 514.28
cx, cy = 321.10, 244.58
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))  # Replace with actual distortion if available

# ==== ArUco marker sizes by ID (in meters) ====
marker_sizes = {
    1: 0.038,  # 3.8cm
    2: 0.086   # 8.6cm
}

# ==== Load YOLOv8 model ====
model = YOLO("/home/tson/Robot/Computer vision/OAK-D/best.pt")

# ==== Setup OAK-D pipeline ====
pipeline = dai.Pipeline()
cam_rgb = pipeline.createColorCamera()
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
cam_rgb.setFps(30)

xout = pipeline.createXLinkOut()
xout.setStreamName("video")
cam_rgb.preview.link(xout.input)

# ==== ArUco setup ====
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# ==== Run device ====
with dai.Device(pipeline) as device:
    video = device.getOutputQueue(name="video", maxSize=4, blocking=False)

    while True:
        in_frame = video.get()
        frame = in_frame.getCvFrame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- ArUco Detection ---
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                id_marker = ids[i][0]
                if id_marker not in marker_sizes:
                    continue

                marker_length = marker_sizes[id_marker]
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[i]], marker_length, camera_matrix, dist_coeffs)

                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec[0], tvec[0], 0.03)
                t = tvec[0][0]
                print(f"[ArUco] ID {id_marker} - Position (x, y, z): {t.round(3)} m")

        # --- YOLOv8 Detection ---
        results = model.predict(source=frame, conf=0.5, imgsz=640, verbose=False)
        annotated_frame = results[0].plot()

        # Overlay ArUco pose info on YOLO frame
        overlay = cv2.addWeighted(annotated_frame, 0.9, frame, 0.1, 0)

        cv2.imshow("YOLO + ArUco Pose", overlay)
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
