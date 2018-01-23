clc;close all;clear all;
load TestTrack

x0=[287,5,-176,0,2,0]';

dt = 0.01;
delta_s=0.2;
N=1/delta_s;
Ndec=2*N+(1+6+1)*(N+1);

U = []; % set of inputs
T = 0; % set of time 
Num_sect = size(TestTrack.bl,2);
%%
% try
for l = 1:(Num_sect-1)
    z0=zeros(Ndec,1);
    for i = 1:N+1
        z0(2*N+8*(i-1)+(2:7))=x0;
    end
    z0(2*N+8:8:end)=1; % initial guess of b
    x_prev=x0;

    Aeq=zeros(1+6,Ndec); %linear equality constrain to set the initial condition
    Aeq(1:7,2*N+1:2*N+7)=eye(7);
    beq=[0;x_prev];

%     theta1 = min(TestTrack.theta(l+1),TestTrack.theta(l+2));
%     theta2 = max(TestTrack.theta(l+1),TestTrack.theta(l+2));
    
    
    if(l<=Num_sect-5)
        theta = TestTrack.theta(l+[0:5])';
    else
        theta = TestTrack.theta(l+[0,0,0])'; % the section before terminal
    end

    if(l<=Num_sect-2)        
        Pc_plus = TestTrack.cline(:,l+2);
    else
        Pc_plus = TestTrack.cline(:,l+1); % the section before terminal
    end
    
    lb=-inf*ones(Ndec,1); %lower bound
    lb(1:2:2*N)=-0.5;     %lb and ub applied for inputs
    lb(2:2:2*N)=-10000;
    lb((2*N+8):8:end)=0; % b >= a
    ub=inf*ones(Ndec,1); %Upper bound
    ub(1:2:2*N)=0.5;
    ub(2:2:2*N)=5000;
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
    bk=zeros(3,1);
    bk(1)=Ak(1,:)*P1;
    bk(2)=Ak(2,:)*P2;
    bk(3)=Ak(3,:)*P3;

    s=sign(Ak*Pc-bk); % The point should have the same sign as the center point to be inside the box

    A=zeros(3*(N+1),Ndec);
    b=zeros(3*(N+1),1);
    for k=1:1:N+1          % linear inequality constrain make sure the point is inside the box
        A(1+3*(k-1):3*k,2*N+8*(k-1)+[2,4])=-s.*Ak;
        b(1+3*(k-1):3*k)=-s.*bk;
    end
    A(end-1,:)=-A(end-1,:);
    b(end-1,:)=-b(end-1,:);
%%%

    % options=optimoptions('fmincon','Display','iter','SpecifyObjectiveGradient',true,...
    % 'SpecifyConstraintGradient',true,'CheckGradients',false,'FiniteDifferenceType','central');
    options=optimoptions('fmincon','Display','iter','SpecifyObjectiveGradient',true,...
    'SpecifyConstraintGradient',true,'CheckGradients',false,'FiniteDifferenceType','central',...
    'UseParallel',true);

    options.MaxFunctionEvaluations=5000;
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
    plot(x,y,'b-','linewidth',2);
    hold on;
    plot(x_real,y_real,'r-','linewidth',1);
    V = [P1';P2';P3';P4'];
    F = [1 2 3 4];
    patch('Faces',F,'Vertices',V,'FaceColor','none')
    axis equal;
    title(['vel: ',num2str(x0_plus(2)),' dT: ',num2str(t_discrete(end)),...
        ' T: ',num2str(T)])
    drawnow;
    %update 
    U = [U;u_discrete];
    T = T + t_discrete(end);
    x0 = x0_plus
%     pause;
end
% 
% catch
%     x0
%     x0 = x_prev
%     l = l-1
% end
