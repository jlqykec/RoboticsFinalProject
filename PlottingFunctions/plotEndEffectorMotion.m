function plotEndEffectorMotion(T6Traj)
%Get the Position of the end effector and the approaching direction
pos=reshape(T6Traj(1:3,4,:),3,[]);
appro=reshape(T6Traj(1:3,3,:),3,[]);

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
