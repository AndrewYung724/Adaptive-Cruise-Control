function accel= impPathV2(vref,Ts,ego_vel,L_RDis,L_Vel,R_RDis,R_Vel)
% #1 From a finite acceleration action space select the an acceleration
% #2 Choose and acceleration and check cost function over next 10 sec
a=3.5; %Assume a max acceration of a m/s^2
a_space=-1.2*a:0.1:a;
T=1;    %Projection time of T seconds
t=0:Ts:T;
Mt=repmat(t',1,length(a_space));

eps=10^-6;

%Ego Velocity matrix
V1=t'*a_space+ego_vel;
%TAKE INTO ACCOUNT CHANGE IN DISTANCE FOR Lead and rear
dL=max(L_RDis+(L_Vel-V1).*Mt,eps);
dR=max(R_RDis+(V1-R_Vel).*Mt,eps);

%apply cost function to each velocity
a=0.0005;     %Extends the TTC explosion region
b=5000;
Fcost=min(eps,L_Vel-V1).^2 ./ (a*dL.^2);
Rcost=min(eps,V1-R_Vel+0.1).^2 ./ (a*dR.^2);
Scost=b*max(vref-V1,0);

J=Fcost+Rcost+Scost;

%Sum each column to attain cost along trajectory for each acceleration
Js=sum(J,1);
[~,i]=min(Js);
 
accel =a_space(i);
 