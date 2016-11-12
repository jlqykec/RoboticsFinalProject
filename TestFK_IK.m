%% Program to test the forward and inverse kinematics of the standford robot
clear all
clc
%Create the StandfordRobot object
Robot=StandfordRobot();

%% Test the forward kinematics
%Four solution for th1=120, th2=45, th3=10, th4=45, th5=26, th6=0

%Input joint variables
q(1,1)=-120*pi/180;
q(2,1)=40*pi/180;
q(3,1)=15;
q(4,1)=0*pi/180;
q(5,1)=-30*pi/180;
q(6,1)=110*pi/180;
%Forward kinematics method
T=Robot.fwKin(q)

%Inverse kinematics method
th4=q(4,1)

tic
Q=Robot.invKin(T);
toc

%Forward kinematics method
T2=Robot.fwKin(Q(1,:)')
sum(sum(T2-T))

Q=Q*180/pi;
Q(:,3)=Q(:,3)*pi/180