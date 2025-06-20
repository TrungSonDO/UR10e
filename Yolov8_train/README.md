# Connect to OAK-D USB Camera
### 1. Install dependencies
```
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
```
- Detailed instructions at [Luxonis Docs](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide/)
### 2. Check camera with DAI Viewer
Luxonis provides a visual tool to check the signal from the camera:
- Download and use DepthAI Viewer at: [Luxonis Docs](https://docs.luxonis.com/software/tools/dai-viewer/)

# YOLOv8 Object Detection And Instance Segmentation with OAK-D Camera 
This section explains how to train a YOLOv8 model to detect boxe, pipe, base link of UR10e then run it in real-time using an OAK-D camera
## Steps Overview
### 1. Data Collection
- Capture ~400 images of the box under various lighting and angles using phone or OAK-D
  > I have prepared the capture_image.py file if you want to take a photo with the camera
### 2. Labeling
- Upload images to [Roboflow](https://roboflow.com)
- Annotate boxe, pipe, base_link, export in YOLOv8 format, and download the dataset

### 3. Upload to Google Drive
- Place the dataset folder inside Yolov8/ on your Drive
- Create a new file: yolov8_training.ipynb in folder Yolov8 and training on Google Colab
  > You get the best.py file after completion and download it to your computer

#
<p align="center">
  <img src="https://github.com/user-attachments/assets/cbb314e8-c010-4b06-8f8b-d2151c279700" alt="Result 1" style="width: 49%; height: 300px; object-fit: contain; margin-right: 1%;" />
  <img src="https://github.com/user-attachments/assets/797d843d-ae8a-448a-b837-2584a42bcb76" alt="Result 2" style="width: 49%; height: 300px; object-fit: contain;" />
</p>
<h5 align="center">YOLOv8 model results after training on Roboflow</h5>

