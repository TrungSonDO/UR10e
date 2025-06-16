import depthai as dai
import cv2
import os

save_dir = "/home/tson/Robot/Computer vision/OAK-D/Images"
os.makedirs(save_dir, exist_ok=True)

pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)

xout = pipeline.createXLinkOut()
xout.setStreamName("rgb")
cam.preview.link(xout.input)

with dai.Device(pipeline) as device:
    queue = device.getOutputQueue("rgb", maxSize=4, blocking=False)
    i = 0

    while True:
        frame = queue.get().getCvFrame()
        cv2.imshow("Capture", frame)

        key = cv2.waitKey(1)
        if key == ord('s'):
            filename = os.path.join(save_dir, f"img_{i}.jpg")
            cv2.imwrite(filename, frame)
            print("Saved:", filename)
            i += 1
        elif key == ord('q'):
            break