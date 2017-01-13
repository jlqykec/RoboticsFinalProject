function Motion=PathPlanJointSpace(qA,qB,qC,TAB,TBC,tacc,ts)
%% First part of the path: from A to a point A'
%Time from 0 to a point A' before the transition
tA_Ap=0:ts:TAB-tacc;

%Calculate the velocity at point A (We assume at point A we have velocity).
deltaB=qA-qB;
VAB=-deltaB/TAB;

%Get the steps for the first part of the path
h=tA_Ap/TAB;
n1=length(h);

%Use the constant velocity trajectory
for i=1:n1
    q(:,i)=-deltaB*h(i)+qA;
    dq(:,i)=VAB;
    ddq(:,i)=[0;0;0;0;0;0];    
end

%% Second part of the path: transition from line AB to line BC
%Calculate deltaB and deltaC
deltaB=q(:,end)-qB;
%deltaB=qA-qB;
deltaC=qC-qB;

%Find h for the transition part
t_trans_BC=-tacc:ts:tacc;
T1=TBC;

%Get the steps for the second part of the path
h=(t_trans_BC+tacc)/(2*tacc);
n2=length(h);

%Use the transition trajectory
for i=1:n2
   h2=h(i)^2;
   q2(:,i)=((deltaC*tacc/T1+deltaB)*(2-h(i))*h2-2*deltaB)*h(i)+qB+deltaB;
   dq2(:,i)=((deltaC*tacc/T1+deltaB)*(1.5-h(i))*2*h2-deltaB)/tacc;
   ddq2(:,i)=(deltaC*tacc/T1+deltaB)*(1-h(i))*3*h(i)/(tacc^2);
end
%Store the path of part 2 in the total path
q=[q q2(:,2:end)];
dq=[dq dq2(:,2:end)];
ddq=[ddq ddq2(:,2:end)];


%% Third part of the path: from B to C
tB_C=tacc:ts:TBC;
T1=TBC;
VBC=deltaC/TBC;

%Get the steps for the third part of the path
h=tB_C/T1;
n3=length(h);

%Use the linear velocity trajectory
for i=1:n3
    q3(:,i)=deltaC*h(i)+qB;
    dq3(:,i)=VBC;
    ddq3(:,i)=[0;0;0;0;0;0];    
end

%Store the path of part 3 in the total path
q=[q q3(:,2:end)];
dq=[dq dq3(:,2:end)];
ddq=[ddq ddq3(:,2:end)];

%Create a time vector for the whole path
t=0:ts:TAB+TBC;

Motion.q=q;
Motion.dq=dq;
Motion.ddq=ddq;
Motion.time=t;

end