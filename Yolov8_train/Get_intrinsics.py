import depthai as dai

with dai.Device() as device:
    calib = device.readCalibration()

    # Get RGB camera parameters (can be changed to LEFT or RIGHT if needed)
    width, height = 640, 480
    intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height)


    # get matrix camera (fx, fy, cx, cy)
    fx = intrinsics[0][0]
    fy = intrinsics[1][1]
    cx = intrinsics[0][2]
    cy = intrinsics[1][2]

    print("Camera intrinsics (for resolution 640x480):")
    print(f"fx = {fx}, fy = {fy}, cx = {cx}, cy = {cy}")

    # affiche
    print("\nCamera Matrix:")
    print(intrinsics)

    # Get distortion factor 
    dist = calib.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
    print("\nDistortion Coefficients:")
    print(dist)
