%% Program to test the forward and inverse kinematics of the standford robot
clear all
clc
%Create the StandfordRobot object
Robot=StanfordRobot();

%% Test point 1
disp('-----------Test point 1---------')
T=[0,0,-1,8;-1,0,0,0;0,1,0,22;0,0,0,1]
%%%%%%%%%%%%%%%%%%%%%%% Inverse kinematics %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Find the inverse kinematics using the StandforRobot object
Q=Robot.invKin(T);

disp('Inverse Kinematics solutions for Test point 1 in degrees')
%Transform to degrees to show the joint results
R=Q*180/pi;
R(:,3)=R(:,3)*pi/180 %The prismatic joint is not an angle so return it to its original value

%%%%%%%%%%%%%%%%%%%%%%%% Forward kinematics %%%%%%%%%%%%%%%%%%%%%%%%%%%
%Use one of the solutions to test the forward kinematics
disp('Forward kinematics of one of the solutions of test point 1')

%Output of the forward kinematics in the Cartesian form
disp('--Solution in the form (n,o,a,p) ')
T1R_Cart=Robot.fwKin(Q(1,:)','Cart')

%Output of the forward kinematics in the RPY form for the angles
disp('--Solutions in the form (x,y,z,phi,theta,psi)')
T1R_RPY=Robot.fwKin(Q(1,:)','RPY');
%Transform the RPY angles to degrees
T1R_RPY(1:2,4:6)=T1R_RPY(1:2,4:6)*180/pi


%% Test point 2
disp('-----------Test point 2---------')
T=[0,0,1,-9;0,1,0,15;-1,0,0,8;0,0,0,1]
%%%%%%%%%%%%%%%%%%%%%%% Inverse kinematics %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Find the inverse kinematics using the StandforRobot object
Q=Robot.invKin(T);

disp('Inverse Kinematics solutions for Test point 2 in degrees')
%Transform to degrees to show the joint results
R=Q*180/pi;
R(:,3)=R(:,3)*pi/180 %The prismatic joint is not an angle so return it to its original value

%%%%%%%%%%%%%%%%%%%%%%%% Forward kinematics %%%%%%%%%%%%%%%%%%%%%%%%%%%
%Use one of the solutions to test the forward kinematics
disp('Forward kinematics of one of the solutions of test point 1')

%Output of the forward kinematics in the Cartesian form
disp('--Solution in the form (n,o,a,p) ')
T2R_Cart=Robot.fwKin(Q(1,:)','Cart')

%Output of the forward kinematics in the RPY form for the angles
disp('--Solutions in the form (x,y,z,phi,theta,psi)')
T2R_RPY=Robot.fwKin(Q(1,:)','RPY');
%Transform the RPY angles to degrees
T2R_RPY(1:2,4:6)=T2R_RPY(1:2,4:6)*180/pi

