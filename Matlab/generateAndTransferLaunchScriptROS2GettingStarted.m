% Remember add path before run this function 
% For example, before run this function, i added this command: addpath('~/Documents/MATLAB/Examples/R2024a/urseries/GettingStartedControllingSimulatedUR10eExample')  
%               to declare this function to be used for ur10e

function generateAndTransferLaunchScriptGettingStarted(device,WorkSpaceFolder,interface,RobotAddress)
    
    % Open a file to write set of commands to lauunch interface with
    % simulated UR10 in gazebo or URSim
    fid=fopen(fullfile(tempdir,"launchUR.sh"),"w+");
    fprintf(fid,"#!/bin/sh\n");
    fprintf(fid,"export SVGA_VGPU10=0\n");
    % Dont need ROS_MASTER_URI and ROS2
    fprintf(fid,"export ROS_DOMAIN_ID=0\n");
    fprintf(fid,"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n");


    if isequal(interface,'Gazebo')
        fprintf(fid,"gnome-terminal --title=\42Simulated UR10 Robot\42 -- /bin/bash -c 'source %s/setup.bash; source %s/devel/setup.bash; roslaunch ur_gazebo ur10e_bringup.launch'",device.ROSFolder,WorkSpaceFolder);
    elseif isequal(interface,'URSim')
        fprintf(fid,"gnome-terminal --title=\42Simulated UR10 Robot\42 -- /bin/bash -c 'source %s/setup.bash; source %s/devel/setup.bash; roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.1.101'",device.ROSFolder,WorkSpaceFolder);
    elseif isequal(interface,'Hardware')
        fprintf(fid,"gnome-terminal --title=\42UR5e Robot Driver\42 -- /bin/bash -c 'source %s/setup.bash; source %s/devel/setup.bash; roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=%s'",device.ROSFolder,WorkSpaceFolder,RobotAddress);
    end
    
    fclose(fid);

    % Copy file into ROS device
    putFile(device,fullfile(tempdir,'launchUR.sh'),'~/')

    % Make the shell script executable
    system(device,'chmod a+x ~/launchUR.sh');
end
