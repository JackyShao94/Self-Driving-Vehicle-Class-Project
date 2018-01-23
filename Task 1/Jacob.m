syms t F_x d_f
x=sym('x',[6,1])

W=13720;
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=2921;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;

% d_f=@(t) interp1(T,U(:,1),t,'previous','extrap');
% F_x=@(t) interp1(T,U(:,2),t,'previous','extrap');

%slip angle functions in degrees
a_f= (d_f-atan2(x(4)+a*x(6),x(2)))*(180/pi) ;
a_r= -atan2((x(4)-b*x(6)),x(2))*(180/pi) ;

%Nonlinear Tire Dynamics
phi_yf= (1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr= (1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_yf= Dy*sin(Cy*atan(By*phi_yf))+Svy;
F_yr= Dy*sin(Cy*atan(By*phi_yr))+Svy;

%vehicle dynamics
df= [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*W+Nw*F_x-F_yf*sin(d_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(d_f)+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(d_f)-F_yr*b)/Iz];
      
 