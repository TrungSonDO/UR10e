# Requirements
- Ubuntu

# Installation
1. Install the application **SVCapture64** from the link below:  
   [SVCapture64 Download](https://www.svs-vistek.com/en/industrial-cameras/svs-camera-detail.php?id=exo174CU3)

# Notes
- After finishing the installation of **SVCapture64**, verify the connection between your PC and the camera.  
- To run a Python file with this industrial camera, place your Python script inside the **/cti** package (included in the unzipped installation files).
- In my case, i run my Python script with this command in terminal:
```
sudo LD_LIBRARY_PATH="/home/tson/Stage/Camera exo 174CU3/SVCam_Kit_v2.5.15_Setup_AMD64_Linux/SVCam_Kit_v2.5.15_Setup_AMD64_Linux/SVCamKit/SDK/Linux64_x64:$LD_LIBRARY_PATH" python3 exo174cu3_swarm2.py
```
   > Change the path of file
- Use the file exo174cu3_swarm2.py to start the camera and begin recording. To stop and save the video, press the q key.


