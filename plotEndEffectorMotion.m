function plotEndEffectorMotion(P1,a)
%Get the number of points of the path
n=length(P1);
figure(4)
%Create a second point to plot a line for each point P
P2=zeros(3,n); %Initialize the set of points
for i=1:n
   P2(:,i)=P1(:,i)+5*a(:,i);

   Points=[P1(:,i)';P2(:,i)'];
   
   plot3(Points(:,1),Points(:,2),Points(:,3));
   hold on   
end

end