%% Trajectory planning for the Stanford robot
clear all
clc

%Add the paths for the StandfordRobot and trajectory planning
addpath StanfordRobot
addpath TrajectoryPlanning

%Add path for the plotting functions
addpath PlottingFunctions

%Create the StandfordRobot object
Robot=StanfordRobot();
%% Starting, Via Point and End Point

%In centimeters
PA=[ 0, 0,-1,  8;
    -1, 0, 0,  0;
     0, 1, 0, 22;
     0, 0, 0,  1];

PB=[ 0, 0, 1, -9;
     0, 1, 0, 15;
    -1, 0, 0,  8;
     0, 0, 0,  1];

PC=[ 0,-1, 0,  5;
     0, 0, 1, 12;
    -1, 0, 0, -2;
     0, 0, 0,  1];

% %In inches
% PA=[ 0, 0,-1, 8/2.54;
%     -1, 0, 0,  0;
%      0, 1, 0, 22/2.54;
%      0, 0, 0,  1];
% 
% PB=[ 0, 0, 1, -9/2.54;
%      0, 1, 0, 15/2.54;
%     -1, 0, 0,  8/2.54;
%      0, 0, 0,  1];
% 
% PC=[ 0,-1, 0,  5/2.54;
%      0, 0, 1, 12/2.54;
%     -1, 0, 0, -2/2.54;
%      0, 0, 0,  1];
 
 %Times
 TAB=0.5;
 TBC=0.5;
 tacc=0.2;
 ts=0.002;
 
 %% Trajectory planning joint motion
%Point A
%Find the inverse kinematics using the StandforRobot object
Q=Robot.invKin(PA);
%Use just the first solution
qA=Q(1,:)';

%Point B
%Find the inverse kinematics using the StandforRobot object
Q=Robot.invKin(PB);
%Use just the first solution
qB=Q(1,:)';

%Point C
%Find the inverse kinematics using the StandforRobot object
Q=Robot.invKin(PC);
%Use just the first solution
qC=Q(1,:)';

%Get the motion planning and store it in the Motion structure
Motion=PathPlanJointSpace(qA,qB,qC,TAB,TBC,tacc,ts);

%% Plot the evolution in the joint space
plotJointEvolution(Motion)

%% Plot the end effector motion

%Find the number of points in the trajectory
n=length(Motion.q);

%Create an empty set of matrices to store the cartesian trajectory given by
%the T6 matrices
T6Traj=zeros(4,4,n);

%Loop through each point in the joint space and use the forward kinematics 
%to find the transformation matrix T6
for i=1:n
    %Calculate the forward kinematics and store the T6 matrix for each
    %point
    T6Traj(:,:,i)=Robot.fwKin(Motion.q(:,i),'Cart');
end

%Plot the cartesian motion
plotEndEffectorMotion(T6Traj,PA,PB,PC);

