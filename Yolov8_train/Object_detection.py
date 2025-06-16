import cv2
import depthai as dai
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO("/home/tson/Robot/Computer vision/OAK-D/best.pt")  # <-- MODIFY YOUR PATH

# Create pipeline OAK-D
pipeline = dai.Pipeline()
cam_rgb = pipeline.createColorCamera()
xout_rgb = pipeline.createXLinkOut()
xout_rgb.setStreamName("video")
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
cam_rgb.setFps(30)
cam_rgb.preview.link(xout_rgb.input)

# Run pipeline
with dai.Device(pipeline) as device:
    video = device.getOutputQueue(name="video", maxSize=4, blocking=False)
    
    while True:
        in_frame = video.get()
        frame = in_frame.getCvFrame()

        # Use YOLOv8 for detecte
        results = model.predict(source=frame, conf=0.5, imgsz=640, verbose=False)

        #  show results on frame
        annotated_frame = results[0].plot()

        # show with opencv
        cv2.imshow("YOLOv8 + OAK-D", annotated_frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()