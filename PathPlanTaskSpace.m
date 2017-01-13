function Motion=PathPlanTaskSpace(PA,PB,PC,TAB,TBC,tacc,ts)
%% Motion from PA to PA'
[x,y,z,theta,phi,psi]=getDriveParam(PA,PB);

%% First part of the path: from PA to a point PA'
%Time from 0 to a point A' before the transition
tA_Ap=0:ts:TAB-tacc;

%Get the steps for the first part of the path
r=tA_Ap/TAB;
n1=length(r);

%Create a set of empty matrices to store the transformation matrices
T6=zeros(4,4,n1);

%Use the drive transformation to find the motion from A to A'
for i=1:n1
    rx=r(i)*x;
    ry=r(i)*y;
    rz=r(i)*z;
    rphi=r(i)*phi;
    rtheta=r(i)*theta;
    
    %Find the drive transformation for each step
    D=driveTransf(rx,ry,rz,rtheta,rphi,psi);
    %Find the transformation matrix for each step
    T6(:,:,i)=PA*D;  
end
%Get the transformation matrix PAp
PAp=reshape(T6(:,:,end),4,4);

%% Motion from PA' to PB and PB to PC
[xA,yA,zA,thetaA,phiA,psiA]=getDriveParam(PB,PAp);
[xC,yC,zC,thetaC,phiC,psiC]=getDriveParam(PB,PC);

%Check for the condition of psiC and psiA
if abs(psiC-psiA)>pi/2
    psiA=psiA+pi;
    thetaA=-1*thetaA;    
end

%% Second part of the path: transition from line AB to line BC

%Assign deltaB and deltaC
deltaB=[xA;yA;zA;thetaA;phiA];
deltaC=[xC;yC;zC;thetaC;phiC];

%Find h for the transition part
t_trans=-tacc:ts:tacc;
T1=TBC;

%Get the steps for the second part of the path
h=(t_trans+tacc)/(2*tacc);
n2=length(h);

%Create a set of empty matrices for the second part of the path
P2=zeros(4,4,n2);

%Create a set of empty vectors for rx,ry,rz,rtheta,rphi
q2=zeros(5,n2);

%Use the transition trajectory
for i=1:n2
   h2=h(i)^2;
   q2(:,i)=((deltaC*tacc/T1+deltaB)*(2-h(i))*h2-2*deltaB)*h(i)+deltaB;
   
   %Find the drive transformation for each step
   D=driveTransf(q2(1,i),q2(2,i),q2(3,i),q2(4,i),q2(5,i),psiA);
   %Find the transformation matrix for each step
   P2(:,:,i)=PB*D;
end
%Store the path of part 2 in the total path
T6=cat(3,T6,P2(:,:,2:end));

%% Third part of the path: from PB to PC
tB_C=tacc:ts:TBC;
T1=TBC;

%Get the steps for the third part of the path
h=tB_C/T1;
n3=length(h);

%Create a set of empty matrices for the third part of the path
P3=zeros(4,4,n3);

%Create a set of empty vectors for rx,ry,rz,rtheta,rphi
q3=zeros(5,n3);

%Use the linear velocity trajectory
for i=1:n3
    q3(:,i)=deltaC*h(i);
    %Find the drive transformation for each step
    D=driveTransf(q3(1,i),q3(2,i),q3(3,i),q3(4,i),q3(5,i),psiC);
    %Find the transformation matrix for each step
    P3(:,:,i)=PB*D;
end
%Store the path of part 3 in the total path
T6=cat(3,T6,P3(:,:,2:end));

%Create a time vector for the whole path
t=0:ts:TAB+TBC;


%Calculate the velocities and accelerations
%Get the position X,Y and Z from the Transformation matrices
x=reshape(T6(1,4,:),1,[]);
y=reshape(T6(2,4,:),1,[]);
z=reshape(T6(3,4,:),1,[]);

%Calculate the velocities
dx=diff(x)/ts;
dy=diff(y)/ts;
dz=diff(z)/ts;

%Calculate the accelerations
ddx=diff(dx)/ts;
ddy=diff(dy)/ts;
ddz=diff(dz)/ts;

%Store everything in the Motion structure
Motion.T6=T6;
Motion.x=x;
Motion.y=y;
Motion.z=z;
Motion.dx=dx;
Motion.dy=dy;
Motion.dz=dz;
Motion.ddx=ddx;
Motion.ddy=ddy;
Motion.ddz=ddz;
Motion.time=t;

end

function D=driveTransf(rx,ry,rz,rtheta,rphi,psi)
    D=zeros(4,4);
    %Find the trigonometric functions first
    Srphi=sin(rphi);
    Crphi=cos(rphi);
    
    Srtheta=sin(rtheta);
    Crtheta=cos(rtheta);
    Vrtheta=1-Crtheta;
    
    Spsi=sin(psi);
    Cpsi=cos(psi);    
    
    %Orientation
    D(1,2)=-Srphi*(Spsi^2*Vrtheta+Crtheta)-Crphi*Spsi*Cpsi*Vrtheta;
    D(2,2)=Srphi*Spsi*Cpsi*Vrtheta+Crphi*(Cpsi^2*Vrtheta+Crtheta);
    D(3,2)=Srphi*Cpsi*Srtheta-Crphi*Spsi*Srtheta;
    
    %Approach
    D(1,3)=Cpsi*Srtheta;
    D(2,3)=Spsi*Srtheta;
    D(3,3)=Crtheta;

    %Normal
    D(1:3,1)=cross(D(1:3,2),D(1:3,3));
    
    %Position
    D(:,4)=[rx;ry;rz;1];  
end











