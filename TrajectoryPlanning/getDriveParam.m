function [x,y,z,theta,phi,psi]=getDriveParam(PA,PB)
%% Extract the vectors from PA and PB
PAn=PA(1:3,1);  PBn=PB(1:3,1);  
PAo=PA(1:3,2);  PBo=PB(1:3,2);  
PAa=PA(1:3,3);  PBa=PB(1:3,3);  
PAp=PA(1:3,4);  PBp=PB(1:3,4);  
PABnn=dot(PAn,PBn);
PABno=dot(PAn,PBo);
PABna=dot(PAn,PBa);
PABon=dot(PAo,PBn);
PABoo=dot(PAo,PBo);
PABoa=dot(PAo,PBa);
PABan=dot(PAa,PBn);
PABao=dot(PAa,PBo);
PABaa=dot(PAa,PBa);
PBAp=PBp-PAp;

%Find x,y,z
x=dot(PAn,PBAp);
y=dot(PAo,PBAp);
z=dot(PAa,PBAp);

%Find psi
psi=atan2(PABoa,PABna);
Spsi=sin(psi);
Cpsi=cos(psi);

%Find theta
theta=atan2(sqrt(PABna^2+PABoa^2),PABaa);
Stheta=sin(theta);
Ctheta=cos(theta);
Vtheta=1-Ctheta;

%Find phi
Sphi=-Spsi*Cpsi*Vtheta*PABnn+(Cpsi^2*Vtheta+Ctheta)*PABon-Spsi*Stheta*PABan;
Cphi=-Spsi*Cpsi*Vtheta*PABno+(Cpsi^2*Vtheta+Ctheta)*PABoo-Spsi*Stheta*PABao;
phi=atan2(Sphi,Cphi);

end
