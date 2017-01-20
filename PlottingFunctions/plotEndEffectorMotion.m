function plotEndEffectorMotion(T6Traj,PA,PB,PC)

%Get the Position of the end effector and the approaching direction
pos=reshape(T6Traj(1:3,4,:),3,[]);
appro=reshape(T6Traj(1:3,3,:),3,[]);

%Plot the end effector position only
figure()
scatter3(pos(1,:),pos(2,:),pos(3,:),'.'); hold on
grid on
xlabel('x(cm)');
ylabel('y(cm)');
zlabel('z(cm)');
title('End effector position');
scatter3(PA(1,4),PA(2,4),PA(3,4),'o','r'); 
scatter3(PB(1,4),PB(2,4),PB(3,4),'o','r'); 
scatter3(PC(1,4),PC(2,4),PC(3,4),'o','r'); 
text(PA(1,4),PA(2,4),PA(3,4),'A(8,0,22)');
text(PB(1,4),PB(2,4),PB(3,4),'B(-9,15,8)');
text(PC(1,4),PC(2,4),PC(3,4),'C(5,12,-2)');

figure()
%plot the path and orientation of the end effector
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
title('End effector position and orientation');
%Mark the starting, via and end points in the plot
text(PA(1,4),PA(2,4),PA(3,4),'A(8,0,22)');
text(PB(1,4),PB(2,4),PB(3,4),'B(-9,15,8)');
text(PC(1,4),PC(2,4),PC(3,4),'C(5,12,-2)');


