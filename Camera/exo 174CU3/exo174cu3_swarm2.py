#!/usr/bin/env python3
import cv2
import numpy as np
from harvesters.core import Harvester
import time

# -------------------------------
# CONFIG: path to your .cti file
# -------------------------------
CTI_FILE = "/home/tson/Stage/Camera exo 174CU3/" \
           "SVCam_Kit_v2.5.15_Setup_AMD64_Linux/SVCam_Kit_v2.5.15_Setup_AMD64_Linux/" \
           "SVCamKit/SDK/Linux64_x64/cti/libsv_u3v_tl_x64.cti"

# -------------------------------
# Initialize Harvester
# -------------------------------
h = Harvester()
h.add_file(CTI_FILE)
h.update()

if len(h.device_info_list) == 0:
    exit("No camera detected!")

print("Detected cameras:", h.device_info_list)

# Create image acquirer
ia = h.create(0)
nm = ia.remote_device.node_map

# -------------------------------
# Camera settings
# -------------------------------
ia.remote_device.node_map.TriggerMode.value = 'Off'       # continuous acquisition
ia.remote_device.node_map.PixelFormat.value = 'BayerRG8'  # adjust if needed

# Exposure (manual)
try:
    nm.ExposureAuto.value = 'Off'
    nm.ExposureTime.value = 7000.0  # 7 ms
except Exception as e:
    print("Failed to set ExposureTime:", e)

# Optional: ROI to increase FPS
# try:
#     nm.OffsetX.value = 0
#     nm.OffsetY.value = 0
#     nm.Width.value  = 1024
#     nm.Height.value = 768
# except:
#     pass

ia.start()

# -------------------------------
# Measure actual camera FPS
# -------------------------------
N_measure = 60
t0 = time.time()
frames = 0
while frames < N_measure:
    with ia.fetch(timeout=2000):
        frames += 1
t1 = time.time()
measured_fps = frames / (t1 - t0)
print(f"Measured effective camera FPS: {measured_fps:.1f}")

# -------------------------------
# Video Writer setup (real-time)
# -------------------------------
output_file = "microrobots_detection_real_time.avi"
fourcc = cv2.VideoWriter_fourcc(*'XVID')
frame_width = nm.Width.value
frame_height = nm.Height.value
out = cv2.VideoWriter(output_file, fourcc, measured_fps, (frame_width, frame_height))

# -------------------------------
# Display + detection parameters
# -------------------------------
scale = 0.5   # scale for display
min_area = 50 # filter small noise

try:
    while True:
        with ia.fetch(timeout=5000) as buffer:
            component = buffer.payload.components[0]
            frame = component.data.reshape(component.height, component.width, -1)
            frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_RG2BGR)

            # --- Microrobot detection ---
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
            binary = cv2.dilate(binary, kernel, iterations=1)

            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            filtered_contours = [c for c in contours if cv2.contourArea(c) > min_area]

            display = frame.copy()

            for c in filtered_contours:
                # Smooth contour
                epsilon = 0.01 * cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, epsilon, True)
                cv2.drawContours(display, [approx], -1, (0, 255, 0), 2)

                # Center (red dot)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(display, (cx, cy), 4, (0, 0, 255), -1)

                # Enclosing circle (blue)
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(display, center, radius, (255, 0, 0), 2)

            # --- Save frame to video ---
            out.write(display)

            # --- Show scaled preview ---
            display_resized = cv2.resize(display, (0,0), fx=scale, fy=scale)
            cv2.imshow("Microrobots Swarm Detection", display_resized)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    ia.stop()
    ia.destroy()
    h.reset()
    out.release()
    cv2.destroyAllWindows()
