%% Trajectory planning for the Stanford robot
clear all
clc

%Add the paths for the StandfordRobot and trajectory planning
addpath StanfordRobot
addpath TrajectoryPlanning

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
 
%% Trajectory planning cartesian motion

P1=[-1, 0, 0,  0;
     0, 1, 0, 10 ;
     0, 0,-1, 10;
     0, 0, 0,  1];

P2=[ 0, 0, 1,  0;
     1, 0, 0,  0;
     0, 1, 0, 10;
     0, 0, 0,  1];

P3=[-1, 0, 0, 10;
     0,-1, 0,  0;
     0, 0, 1, 10;
     0, 0, 0,  1];

%Motion=PathPlanTaskSpace(P1,P2,P3,1,1,0.1,ts);
Motion=PathPlanCartesianSpace(PA,PB,PC,TAB,TBC,tacc,ts);
T=Motion.T6;

pos=reshape(T(1:3,4,:),3,[]);
appro=reshape(T(1:3,3,:),3,[]);

%plot the position
figure(1)
%Plot X position, velocity and acceleration
subplot(3,3,1)
plot(Motion.time(),Motion.x)
title('position of x');
grid on
subplot(3,3,2)
plot(Motion.time(1:end-1),Motion.dx)
title('velocity of x');
grid on
subplot(3,3,3)
plot(Motion.time(1:end-2),Motion.ddx)
title('acceleration of x');
grid on

%Plot Y position, velocity and acceleration
subplot(3,3,4)
plot(Motion.time(),Motion.y)
title('position of y');
ylabel('Position(cm)');
grid on
subplot(3,3,5)
plot(Motion.time(1:end-1),Motion.dy)
title('velocity of y');
ylabel('Velocity(cm/s)');
grid on
subplot(3,3,6)
plot(Motion.time(1:end-2),Motion.ddy)
title('acceleration of y');
ylabel('Acceleration(cm/s^2)');
grid on

%Plot Z position, velocity and acceleration
subplot(3,3,7)
plot(Motion.time(),Motion.z)
title('position of z');
grid on
subplot(3,3,8)
plot(Motion.time(1:end-1),Motion.dz)
title('velocity of z');
grid on
subplot(3,3,9)
plot(Motion.time(1:end-2),Motion.ddz)
title('acceleration of z');
grid on

%plot the path and orientation of the end effector
figure(4)
n=length(pos); 
for i=1:n
    P1=pos(:,i);
    P2=P1+appro(:,i)*5;
    v=[P1';P2'];
    plot3(v(:,1),v(:,2),v(:,3)); hold on   
end
grid on
xlabel('x(cm)');
ylabel('y(cm)');
zlabel('z(cm)');
text(PA(1,4),PA(2,4),PA(3,4),'A(8,0,22)');
text(PB(1,4),PB(2,4),PB(3,4),'B(-9,15,8)');
text(PC(1,4),PC(2,4),PC(3,4),'C(5,12,-2)');












