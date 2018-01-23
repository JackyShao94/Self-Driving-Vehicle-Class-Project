function df_v = dyn_test(t,x,d_f,F_x)
% car dynamics for testing
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

%generate input functions
% d_f=@(t) 0;
% F_x=@(t) 0;

%slip angle functions in degrees
a_f=@(t,x,d_f,F_x) rad2deg(d_f-atan2(x(4)+a*x(6),x(2)));
a_r=@(t,x,d_f,F_x) rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=@(t,x,d_f,F_x) (1-Ey)*(a_f(t,x,d_f,F_x)+Shy)+(Ey/By)*atan(By*(a_f(t,x,d_f,F_x)+Shy));
phi_yr=@(t,x,d_f,F_x) (1-Ey)*(a_r(t,x,d_f,F_x)+Shy)+(Ey/By)*atan(By*(a_r(t,x,d_f,F_x)+Shy));

F_yf=@(t,x,d_f,F_x) Dy*sin(Cy*atan(By*phi_yf(t,x,d_f,F_x)))+Svy;
F_yr=@(t,x,d_f,F_x) Dy*sin(Cy*atan(By*phi_yr(t,x,d_f,F_x)))+Svy;

%vehicle dynamics
df=@(t,x,d_f,F_x) [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*W+Nw*F_x-F_yf(t,x,d_f,F_x)*sin(d_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf(t,x,d_f,F_x)*cos(d_f)+F_yr(t,x,d_f,F_x))/m-x(2)*x(6);...
          x(6);...
          (F_yf(t,x,d_f,F_x)*a*cos(d_f)-F_yr(t,x,d_f,F_x)*b)/Iz];
df_v = df(t,x,d_f,F_x);
end