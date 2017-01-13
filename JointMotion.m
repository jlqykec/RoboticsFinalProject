%% Trajectory planning for the Stanford robot
clear all
clc

%Add the paths for the StandfordRobot and trajectory planning
addpath StanfordRobot
addpath TrajectoryPlanning

%Add path for the plottin functions
addpath PlottingFunctions

%Create the StandfordRobot object
Robot=StanfordRobot();
%% Starting, Via Point and End Point
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
PA_q=Q(1,:)';

%Point B
%Find the inverse kinematics using the StandforRobot object
Q=Robot.invKin(PB);
%Use just the first solution
PB_q=Q(1,:)';

%Point C
%Find the inverse kinematics using the StandforRobot object
Q=Robot.invKin(PC);
%Use just the first solution
PC_q=Q(1,:)';

%Get the motion planning and store it in the strcuture Motion
Motion=PathPlanJointSpace(PA_q,PB_q,PC_q,TAB,TBC,tacc,ts);

%% Plot the evolution in the joint space
plotJointEvolution(Motion)

%% Plot the end effector motion

%First use the forward kinematics to find the transformation matrix T6 for
%each point in the trajectory

%Find the number of points in the trajectory
n=length(Motion.q);

%Create an empty set of matrices to store the cartesian trajectory given by
%the T6 matrices
T6Traj=zeros(4,4,n);

for i=1:n
    T6=Robot.fwKin(Motion.q(:,i),'Cart'); %Calculate the forward kinematics
    T6Traj(:,:,i)=T6; %Store the T6 matrix in the cartesian trajectory
end

%Plot the cartesian motion
figure()
plotEndEffectorMotion(T6Traj);
text(PA(1,4),PA(2,4),PA(3,4),'A(8,0,22)');
text(PB(1,4),PB(2,4),PB(3,4),'B(-9,15,8)');
text(PC(1,4),PC(2,4),PC(3,4),'C(5,12,-2)');


