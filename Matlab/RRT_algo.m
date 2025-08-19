% Load robot UR10e
robot = loadrobot("universalUR10e", "DataFormat", "row");

% Box specifications 
boxSize = [0.21, 0.15, 0.12]; % box size (m)
boxCenter = [0.6, 0, 0.3];    % box coordinates relative to robot base

% Create collision box
envBox = collisionBox(boxSize(1), boxSize(2), boxSize(3));
envBox.Pose = trvec2tform(boxCenter);

% Create planner with envBox as obtacle
planner = manipulatorRRT(robot, {envBox});
planner.SkippedSelfCollisions = "parent";
planner.ValidationDistance = 0.05;

% Create workspaceGoalRegion (bottom of box)
bottomFaceOffset = [0, 0, -boxSize(3)/2];
goalPose = envBox.Pose * trvec2tform(bottomFaceOffset);

goalRegion = workspaceGoalRegion(robot.BodyNames{end});
goalRegion.ReferencePose = goalPose;
goalRegion.Bounds(1,:) = [-0.05 0.05];
goalRegion.Bounds(2,:) = [-0.05 0.05];
goalRegion.Bounds(3,:) = [-0.01 0.01];
goalRegion.Bounds(4,:) = [-pi/6 pi/6];
goalRegion.EndEffectorOffsetPose = trvec2tform([0 0 -0.14]);

% show robot and obtacle
figure('Position',[100 100 800 600]);
ax = show(robot);
hold on
show(envBox);
show(goalRegion);

% Plan move
startConfig = homeConfiguration(robot);
rng(0)
path = plan(planner, startConfig, goalRegion);

% Interpolation
interpConfigurations = interpolate(planner, path, 5);

% Show robot moving with appropriate viewport limits
for i = 1:size(interpConfigurations,1)
    show(robot, interpConfigurations(i,:), "PreservePlot", false, "FastUpdate", true);
    axis(ax, 'equal');
    xlim([-0.5 1]);
    ylim([-0.5 0.8]);
    zlim([-1 0.6]);
    view(ax, [60 40]); 
    drawnow
end
hold off
