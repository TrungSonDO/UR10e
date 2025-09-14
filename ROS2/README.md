# Workspace structure 
```
ur_robot_ws/
├── src/
│ ├── ur10e_task_manager/
│ │ ├── package.xml
│ │ ├── CMakeLists.txt
│ │ └── src/
│ │ │  └── ur10e_home_and_up.cpp
│ ├── Universal_Robots_ROS2_Descripiton/
│ └── Universal_Robots_ROS2_Driver/
├── install/
├── build/
└── log/
```

# UR10e End Effector 
<p align="center">
  <img src="https://github.com/user-attachments/assets/9875ce9c-76a8-4ccd-8dda-a51d1c5e97d7" style="width: 49%; height: 300px; object-fit: contain; " />
</p>
<h5 align="center">Component structure</h5>

# Setup system
## Requirements
- ROS2 Humble
- Ubuntu 22.04
- ROS2 packages (See in link below)
```
  https://github.com/UniversalRobots
```
## Network setup
- **Robot UR10e IP :** `192.168.1.101`
- **Machine IP :** `192.168.1.102`
- Check the connection:
```
ping 192.168.1.101
```
  > If ping fails: check Ethernet cable, PC and robot network configuration.
## Quick troubleshooting
- If ros2 launch ur_robot_driver ... fails to connect: check robot_ip and that the external control program is running on the teach pendant.
- If MoveIt does not connect: check use_sim_time and list topics with:
```
ros2 topic list
```
## Terminal
**Step 1:** Build the workspace: 
```
colcon build --symlink-install
source install/setup.bash
```
**Step 2:** Launch the Universal Robots driver (connects to the real UR10e hardware):
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.1.101 use_fake_hardware:=false launch_rviz:=false
```
**Step 3:** On the UR10e teach pendant, open the dashboard and start the external control program.
- On the UR10e teach pendant: go to Program → Open (or Dashboard, depending on firmware), select the external control program, and press Play / Run.
  > Note: the robot must be running this program for ROS2 to control it.
  
**Step 4:** In a new terminal, launch MoveIt for motion planning:
- Open another terminal and run:
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true use_sim_time:=false
```
**Step 5:** In another terminal, run the custom task manager node (example: move robot to home and up position):
```
ros2 run ur10e_task_manager ur10e_home_and_up
```






