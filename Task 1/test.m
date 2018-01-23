
%% Problem 1 Project
%Given a set of inputs and an initial condition, returns the vehicles
%trajectory. If no initial condition is specified the default for the track
%is used.
%
% INPUTS:
%   U           an N-by-2 vector of inputs, where the first column is the
%               steering input in radians, and the second column is the 
%               longitudinal force in Newtons.
%   
%   x0          a 1-by-6 vector of the initial state of the vehicle.
%
% OUTPUTS:
%   Y           an N-by-6 vector where each column is the trajectory of the
%               state of the vehicle
%
% Written by: Matthew Porter
% Created: 13 Nov 2017
% Modified: 16 Nov 2017

% load TestTrack
%if initial condition not given use default
%if nargin<2
%     x0=[287,5,-176,0,2,0]';
%end

%generate time vector
%T=0:0.01:(size(U,1)-1)*0.01;

%constants
% W=13720;
% Nw=2;
% f=0.01;
% Iz=2667;
% a=1.35;
% b=1.45;
% By=0.27;
% Cy=1.2;
% Dy=2921;
% Ey=-1.6;
% Shy=0;
% Svy=0;
% m=1400;

%generate input functions
% d_f=@(t) interp1(T,U(:,1),t,'previous','extrap');
% F_x=@(t) interp1(T,U(:,2),t,'previous','extrap');
% 
% %slip angle functions in degrees
% a_f=@(t,x) rad2deg(d_f(t)-atan2(x(4)+a*x(6),x(2)));
% a_r=@(t,x) rad2deg(-atan2((x(4)-b*x(6)),x(2)));
% 
% %Nonlinear Tire Dynamics
% phi_yf=@(t,x) (1-Ey)*(a_f(t,x)+Shy)+(Ey/By)*atan(By*(a_f(t,x)+Shy));
% phi_yr=@(t,x) (1-Ey)*(a_r(t,x)+Shy)+(Ey/By)*atan(By*(a_r(t,x)+Shy));
% 
% F_yf=@(t,x) Dy*sin(Cy*atan(By*phi_yf(t,x)))+Svy;
% F_yr=@(t,x) Dy*sin(Cy*atan(By*phi_yr(t,x)))+Svy;
% 
% %vehicle dynamics
% df=@(t,x) [x(2)*cos(x(5))-x(4)*sin(x(5));...
%           (-f*W+Nw*F_x(t)-F_yf(t,x)*sin(d_f(t)))/m+x(4)*x(6);...
%           x(2)*sin(x(5))+x(4)*cos(x(5));...
%           (F_yf(t,x)*cos(d_f(t))+F_yr(t,x))/m-x(2)*x(6);...
%           x(6);...
%           (F_yf(t,x)*a*cos(d_f(t))-F_yr(t,x)*b)/Iz];
clear all;
load test_success.mat
dt = 0.01;
delta_s=0.1;
N=1/delta_s;
Ndec=2*N+(1+6+1)*(N+1);

% U = [];
Num_sect = size(TestTrack.bl,2);

%%

% for l = (Num_sect-2):(Num_sect-1)
    z0=zeros(Ndec,1);
    for i = 1:N+1
        z0(2*N+8*(i-1)+(2:7))=x0;
    end
    z0(2*N+8:8:end)=1; % initial guess of b
    x_prev=x0;

    Aeq=zeros(1+6,Ndec);
    Aeq(1:7,2*N+1:2*N+7)=eye(7);
    beq=[0;x_prev];

%     theta1 = min(TestTrack.theta(l+1),TestTrack.theta(l+2));
%     theta2 = max(TestTrack.theta(l+1),TestTrack.theta(l+2));
    theta = TestTrack.theta(l+[0;1]);

    lb=-inf*ones(Ndec,1);
    lb(1:2:2*N)=-0.5;
    lb(2:2:2*N)=-10000;
    lb((2*N+8):8:end)=0; % b >= a
    ub=inf*ones(Ndec,1);
    ub(1:2:2*N)=0.5;
    ub(2:2:2*N)=6000;
%     ub(end-8+3) = 20;
%     lb(end-2) = theta-0.1;
%     ub(end-2) = theta+0.1;
%     lb(end-2) = theta-0.05;
%     ub(end-2) = theta+0.05;

    % Section 1
    P1=TestTrack.bl(:,l);
    P2=TestTrack.bl(:,l+1);
    P3=TestTrack.br(:,l+1);
    P4=TestTrack.br(:,l);
    Ak=zeros(3,2);
    DP1=P1-P2;
    DP2=P2-P3;
    DP3=P3-P4;
    
    
    Ak(1,:)=[DP1(2),-DP1(1)];
    Ak(2,:)=[DP2(2),-DP2(1)];
    Ak(3,:)=[DP3(2),-DP3(1)];

    Pc=TestTrack.cline(:,l);
    Pc_plus = TestTrack.cline(:,l+1);
    bk=zeros(3,1);
    bk(1)=Ak(1,:)*P1;
    bk(2)=Ak(2,:)*P2;
    bk(3)=Ak(3,:)*P3;

    s=sign(Ak*Pc-bk);

    A=zeros(3*(N+1),Ndec);
    b=zeros(3*(N+1),1);
    for k=1:1:N+1
        A(1+3*(k-1):3*k,2*N+8*(k-1)+[2,4])=-s.*Ak;
        b(1+3*(k-1):3*k)=-s.*bk;
    end
    A(end-1,:)=-A(end-1,:);
    b(end-1,:)=-b(end-1,:);
%%%

    % options=optimoptions('fmincon','Display','iter','SpecifyObjectiveGradient',true,...
    % 'SpecifyConstraintGradient',true,'CheckGradients',false,'FiniteDifferenceType','central');
    options=optimoptions('fmincon','Display','iter','SpecifyObjectiveGradient',true,...
    'SpecifyConstraintGradient',true,'CheckGradients',false,'FiniteDifferenceType','central');

    options.MaxFunctionEvaluations=1000;
    % options.MaxIterations = 10000; 
    [X,FVAL,EXITFLAG,OUTPUT]=fmincon(@(z) cost_center(z,Ndec,Pc_plus,theta),z0,A,b,Aeq,beq,lb,ub,@(z) mycon(z,N,delta_s),options);
%%%
    % extract t,u,x,y from optimization result
    [t,u,x,y,v_u,v_v] = ext_result(X,N); 

    % extract real input
    t_discrete = [t(1):0.01:(ceil(t(end)/0.01)*0.01)]'; 
    idx=sum(t_discrete'>=t);% t_discrete is row vector; t is colomn vector
    idx = idx(1:end-1);
    u_discrete = u(idx,:);

    % simulate in current section

    [x0_plus,x_real,y_real] = simu_section(t_discrete,u_discrete,x0);

    % visual
    plot(x,y,'b-','linewidth',20);
    hold on;
    plot(x_real,y_real,'r-','linewidth',10);
    V = [P1';P2';P3';P4'];
    F = [1 2 3 4];
    patch('Faces',F,'Vertices',V,'FaceColor','none')
    axis equal;
    drawnow;
    %update 
    U = [U;u_discrete];
    T = T + t_discrete(end);
    x0 = x0_plus
%     pause;
% end