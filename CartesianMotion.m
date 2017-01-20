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
%  PA=[ 0, 0,-1, 8/2.54;
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
 
%% Trajectory planning cartesian motion

%Get the motion planning and store it in the Motion structure
Motion=PathPlanCartesianSpace(PA,PB,PC,TAB,TBC,tacc,ts);
T6Traj=Motion.T6;

%% Plot the evolution in the cartesian space
plotCartEvolution(Motion);

%% Plot the end effector motion

%Plot the cartesian motion
plotEndEffectorMotion(T6Traj,PA,PB,PC);

