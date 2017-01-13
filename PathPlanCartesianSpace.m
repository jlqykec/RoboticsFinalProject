function Motion=PathPlanCartesianSpace(PA,PB,PC,TAB,TBC,tacc,ts)
%Extract the vectors from PA and PB
%Normal direction
PAn=PA(1:3,1);   PBn=PB(1:3,1);   PCn=PC(1:3,1);
%Approaching direction
PAo=PA(1:3,2);   PBo=PB(1:3,2);   PCo=PC(1:3,2);
%Orientation direction
PAa=PA(1:3,3);   PBa=PB(1:3,3);   PCa=PC(1:3,3);
%Position
PAp=PA(1:3,4);   PBp=PB(1:3,4);   PCp=PC(1:3,4);

%% First find x,y,z,theta,phi,psi for the different segments
%Find x,y,z,theta,phi and psi
%First x,y,z
PABp=PBp-PAp;
x=dot(PAn,PABp);
y=dot(PAo,PABp);
z=dot(PAa,PABp);

%Calculate all the dot points needed
PABnn=dot(PAn,PBn);
PABoo=dot(PAo,PBo);
PABaa=dot(PAa,PBa);
PABna=dot(PAn,PBa);
PABno=dot(PAn,PBo);
PABon=dot(PAo,PBn);
PABoa=dot(PAo,PBa);
PABan=dot(PAa,PBn);
PABao=dot(PAa,PBo);

%psi
psi=atan2(PABoa,PABna);
Spsi=sin(psi);
Cpsi=cos(psi);

%theta
theta=atan2(sqrt(PABna^2+PABoa^2)/PABaa);
Stheta=sin(theta);
Ctheta=cos(theta);
Vtheta=(1-cos(theta));

%phi
Sphi=-Spsi*Cpsi*Vtheta*PABnn+(Cpsi^2*Vtheta+Ctheta)*PABon-Spsi*Stheta*PABan;
Cphi=-Spsi*Cpsi*Vtheta*PABno+(Cpsi^2*Vtheta+Ctheta)*PABoo-Spsi*Stheta*PABao;
phi=atan2(Sphi,Cphi);

%Define the generalized coordinates vector
qA=[0;0;0;0;psi];
qB=[x;y;z;theta;phi;psi];






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
deltaB=q(:,end)-qB; %deltaB goes from A' to B
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

function T=driveTransform(Pos1,r,x,y,z,theta,phi,psi)
    
Srtheta=sin(r*theta);
Crtheta=cos(r*theta);
Vrtheta=1-Crtheta;

Srphi=sin(r*phi);
Crphi=cos(r*phi);

Spsi=sin(r*psi);
Cpsi=cos(r*psi);

D=zeros(4,4);

%Orientation
D(1,2)=-Srphi*(Spsi^2*Vrtheta+Crtheta)+Crphi*(-Spsi*Cpsi*Vrtheta);
D(2,2)=-Srphi*(-Spsi*Cpsi*Vrtheta)+Crphi*(Cpsi^2*Vrtheta+Crtheta);
D(3,2)=-Srphi*(-Cpsi*Srtheta)+Crphi*(-Spsi*Srtheta);

%Approach
D(1,3)=Cpsi*Srtheta;
D(2,3)=Spsi*Srtheta;
D(3,3)=Crtheta;

%Normal
D(1:3,1)=dot(D(1:3,2),D(1:3,3));

%Position
D(1:3,4)=[r*x;r*y;r*z;1];    
    
end









