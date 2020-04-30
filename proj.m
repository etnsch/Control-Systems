clear all
clc

%Initialize map & probabalistic roadmap (prm) for automatic path finding
load exampleMaps
map = binaryOccupancyMap(simpleMap);
figure %figure to display original map
show(map)
mapInflated = copy(map);
prm = robotics.PRM(mapInflated);
startLocation = [4.0 2.0];
endLocation = [23.0 8.0];

%Find path automatically
path = findpath(prm, startLocation, endLocation)
%Manual path
%path = [4.0 2.0;
%        10.0 15.0;
%        17.0 15.0;
%        19.0 4.0;
%        25.0 4.0]

%Initialize robot pose
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';

%Initialize robot
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

%Increases size of the occupied locations in the map
inflate(mapInflated, robot.TrackWidth/2);

%Nodes in path
prm.NumNodes = 100;
prm.ConnectionDistance = 10;

%Initialize controller
controller = controllerPurePursuit;     %Robot controller - returns linear & angular velocity based on current position & orientation
controller.Waypoints = path;            %Path to follow
controller.DesiredLinearVelocity = 1.0; %Max speed
controller.MaxAngularVelocity = 3;      %Turning speed
controller.LookaheadDistance = 0.5;     %Local goal for robot
goalRadius = 0.1;                       %Robot stops when within .1 meters of goal
distanceToGoal = norm(robotInitialLocation - robotGoal);

%Initialize the simulation loop - 10 Hz
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

%Initialize the figure to display robot & path
figure

%Output to gif, need to install 'gif' v1.0.0.0 by Chad Greene
%Sets up gif file to record to, records current state of figure every 0.1 sec
%gif('myfile.gif', 'DelayTime', 0.1, 'frame', gcf)

frameSize = robot.TrackWidth/0.8;

while( distanceToGoal > goalRadius )
    
    %Compute the controller outputs, inputs to the robot
    [linv, angv] = controller(robotCurrentPose);
    
    %Get robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [linv angv]);
    
    %Update current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    %Find new distance to goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    %Update the figure
    hold off
    show(map);
    hold all

    %Plot path instance so it stays persistent while robot mesh moves
    plot(path(:,1), path(:,2),"k--d")
    
    %Plot path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 27])
    ylim([0 26])
    
    %Record gif frame
    %gif
    
    %Wait
    waitfor(vizRate);
end