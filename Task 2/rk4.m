function [Y]=rk4(U,x0)

if nargin<2
    x0=[287,5,-176,0,2,0];
end

%generate time vector
T=0:0.01:(size(U,1)-1)*0.01;

%constants
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
d_f=@(t) interp1(T,U(:,1),t,'previous','extrap');
F_x=@(t) interp1(T,U(:,2),t,'previous','extrap');

%slip angle functions in degrees
a_f=@(t,x) rad2deg(d_f(t)-atan2(x(4)+a*x(6),x(2)));
a_r=@(t,x) rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=@(t,x) (1-Ey)*(a_f(t,x)+Shy)+(Ey/By)*atan(By*(a_f(t,x)+Shy));
phi_yr=@(t,x) (1-Ey)*(a_r(t,x)+Shy)+(Ey/By)*atan(By*(a_r(t,x)+Shy));

F_yf=@(t,x) Dy*sin(Cy*atan(By*phi_yf(t,x)))+Svy;
F_yr=@(t,x) Dy*sin(Cy*atan(By*phi_yr(t,x)))+Svy;

%vehicle dynamics
df=@(t,x) [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*W+Nw*F_x(t)-F_yf(t,x)*sin(d_f(t)))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf(t,x)*cos(d_f(t))+F_yr(t,x))/m-x(2)*x(6);...
          x(6);...
          (F_yf(t,x)*a*cos(d_f(t))-F_yr(t,x)*b)/Iz];
      
%Solve for trajectory 
% % options = odeset('RelTol',1e-6,'AbsTol',1e-10,'NormControl','on');
% options = odeset('Refine',50);
% [~,Y]=ode45(df,T,x0);

% RK4
Y = zeros(length(T),6);
h = 0.01;
% h = t(2)-t(1);
tmp_x = x0;
Y(1,:) = x0;

for i = 1:(length(T)-1)
    tmp_t = T(i+1);
    k1 = df(tmp_t,tmp_x);
    k2 = df(tmp_t+h/2,tmp_x+h*k1/2);
    k3 = df(tmp_t+h/2,tmp_x+h*k2/2);
    k4 = df(tmp_t+h,tmp_x+h*k3);
    tmp_x = tmp_x + h/6*(k1+2*k2+2*k3+k4);
%     tmp_x = tmp_x + h*k1;
    Y(i+1,:)=tmp_x';
end


end
