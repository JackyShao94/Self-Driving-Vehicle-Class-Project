function [x, U_o] = mpc_LTV( AFun, BFun, Q, R, tspan, PredHorizon, AXcons, bXcons, AUcons, bUcons, dx0, Y, U, q)

n = size( dx0, 1 ); %size of state vector
m = size( R, 1 ); % size of input vector
dt = tspan( 2 ) - tspan( 1 ); % assumed fixed time span

x = zeros( length( tspan ), n );
% u = zeros( length( tspan ) - 1 , m ); % don't have an input at the last time step.
U_o = [];

% the first n * PredHorizon variables of the decision variables will correspond to the state 
% the next m * (PredHorizon - 1) variables of the decision variables will correspond to the input

% we begin by defining the cost function
H = zeros(  n * PredHorizon + m * ( PredHorizon - 1 ) );
c = zeros(  n * PredHorizon + m * ( PredHorizon - 1 ), 1 );
% for i = 1%PredHorizon
% %     if i == PredHorizon
% %         Q = [1 0 0 0 0 0;...
% %              0 0 0 0 0 0;...
% %              0 0 1 0 0 0;...
% %              0 0 0 0 0 0;...
% %              0 0 0 0 0 0;...
% %              0 0 0 0 0 0];
% %     else
% %         Q = [0 0 0 0 0 0;...
% %              0 0 0 0 0 0;...
% %              0 0 0 0 0 0;...
% %              0 0 0 0 0 0;...
% %              0 0 0 0 1 0;...
% %              0 0 0 0 0 0];
% %     end
%     H( ( ( i - 1 ) * n + 1 ):( i * n ), ( ( i - 1 ) * n + 1 ):( i * n ) ) = Q;
% end
for i = 1 : ( PredHorizon - 1 )
    H( ( ( i - 1 ) * m + n * PredHorizon + 1 ):( i * m + n * PredHorizon ), ( ( i - 1 ) * m + n * PredHorizon + 1 ):( i * m + n * PredHorizon ) ) = R;
end

% next we define the inequality constraints corresponding to bound
% constraints we have 2 constraints for each state and input at each time
% instance
Aineq = zeros( 2 * n * PredHorizon + 2 * m * (PredHorizon - 1 ), n * PredHorizon + m * ( PredHorizon - 1 ) );
bineq = zeros( 2 * n * PredHorizon + 2 * m * (PredHorizon - 1 ), 1 );
% state constraints


initial_state = dx0;
x( 1, : ) = dx0;
u_prev = [0,0];
u_backup = [];
% tmp_Q = zeros(6);
% tmp_Q(1) = Q(1);

for j = 0:(length( tspan ) - 2 - PredHorizon)
    % redefine first block of H 
    theta = atan2(Y(j+2,3)-Y(j+1,3),Y(j+2,1)-Y(j+1,1));
    Q_plus = [cos(theta) 0 sin(theta) 0 0 0
              0 1 0 0 0 0 
              -sin(theta) 0 cos(theta) 0 0 0
              0 0 0 1 0 0
              0 0 0 0 1 0
              0 0 0 0 0 1];
    Q_plus = (Q_plus'*Q*Q_plus);
    H(1:n,1:n) = (Q_plus + Q_plus')/2;
%     for i = PredHorizon
%         Q_plus = [cos(theta) 0 sin(theta) 0 0 0
%               0 1 0 0 0 0 
%               -sin(theta) 0 cos(theta) 0 0 0
%               0 0 0 1 0 0
%               0 0 0 0 1 0
%               0 0 0 0 0 1];
%         Q_plus = (Q_plus'*Q*Q_plus);
%         H(n*(i-1)+1:n*i,n*(i-1)+1:n*i)= (Q_plus + Q_plus');
%     end
%     
    if j > 0
%                 AUcons{j+1} = [eye(2);-eye(2)];
%                 bUcons{j+1} = [U_o(end,1)*0.2+0.05;
%                       U_o(end,2)*0.05+10;
%                       U_o(end,1)*0.2+0.05;
%                     U_o(end,2)*0.05+10];
%                 U_o(end,2)*0.1
                AUcons{j+1} = [eye(2);-eye(2)];
                bUcons{j+1} = [abs(U_o(end,1))*0.2+0.05;
                            min(abs(U_o(end,2))*0.1+20,bUcons{j+1}(2));
                            abs(U_o(end,1))*0.2+0.05;
                            min(abs(U_o(end,2))*0.1+20,bUcons{j+1}(4))];
    else
                AUcons{j+1} = [eye(2);-eye(2)];
                bUcons{j+1} = [0.01;
                      10;
                      0.01;
                    10];
    end
    for i = 1 : PredHorizon
       Aineq( ( ( i - 1 ) * n * 2 + 1 ):( i * n * 2 ), ( ( i - 1 ) * n + 1 ):( i * n ) ) = AXcons{( j + i)};
       bineq( ( ( i - 1 ) * n * 2 + 1 ):( i * n * 2 ), 1 ) = bXcons{( j + i)};
    end
    % input constraints
    for i = 1:( PredHorizon - 1 )
        Aineq( ( ( i - 1 ) * m * 2 + 2 * n * PredHorizon + 1 ):( i * m * 2 + 2 * n * PredHorizon ), ...
            ( ( i - 1 ) * m + n * PredHorizon + 1 ):( i * m + n * PredHorizon ) ) = AUcons{( j + i)};
        bineq( ( ( i - 1 ) * m * 2 + 2 * n * PredHorizon + 1 ):( i * m * 2 + 2 * n * PredHorizon ), 1 ) = bUcons{( j + i)};
    end
    
    % next we define the equality constraints corresponding to Euler
    % integration one for the initial condition and one for each time step
    % until the next to last time step since the dynamics are time-varying we
    % will have to build this right before we solve the QP
    Aeq = zeros( n * PredHorizon, n * PredHorizon + m * ( PredHorizon - 1 ) );
    beq = zeros( n * PredHorizon, 1 );
    
    
    Aeq( 1:n, 1:n ) = eye( n ); 
    beq( 1:n ) = initial_state;
    for i = 1:( PredHorizon - 1 )
        Aeq( ( i * n + 1 ):( ( i + 1 ) * n ), ( i * n + 1 ):( ( i + 1 ) * n ) ) = -eye( n );
        Aeq( ( i * n + 1 ):( ( i + 1 ) * n ), ( ( i - 1 ) * n + 1 ):( i * n ) ) = dt * AFun{( j + i)} + eye( n );
        Aeq( ( i * n + 1 ):( ( i + 1 ) * n ), ...
            ( ( i - 1 ) * m + PredHorizon * n + 1 ):( i * m + PredHorizon * n ) ) = dt * BFun{( j + i)};
    end
    
    options = optimoptions('quadprog','Display','off');
    [solDV,fval,exitflag,output] = quadprog( H, c, Aineq, bineq, Aeq, beq, [], [], [], options);
    
    if isempty(solDV)
%         keyboard()
        u = u_backup(1,:);
        u_backup(1,:)=[];
        disp('the car is too fast or too slow for unknown reason.');
       
    else
        u = solDV( ( n * PredHorizon + 1 ):( m + n * PredHorizon ) );
        u_backup = [solDV( ( n * PredHorizon + 3 ):2:end),solDV( ( n * PredHorizon + 4 ):2:end)];
    end
%     x( j + 2, : ) = solDV( ( n + 1 ):( 2 * n ) );
%         u = solDV( ( n * PredHorizon + 1 ):( m + n * PredHorizon ) );

%     initial_state = x( j + 2, : );

%     U_d = [ones(q+1,1).*u(1),ones(q+1,1).*u(2)];
    U_d = [linspace(u_prev(1),u(1),q+1)',linspace(u_prev(2),u(2),q+1)'];
    U_d = U_d + U(j*q+1:(j+1)*q+1,:);
    
    U_o = [U_o(1:end-1,:);U_d(1:end,:)];
    
    u_prev = u;
    
%     if j == floor(length( tspan )/4) || j == floor(length( tspan )/2)
%     ode_state = forwardIntegrateControlInput_strong(U_o,Y(1,:));
%     initial_state = ode_state(end,:)'-Y(j+2,:)';
%     x( j + 2, : ) = initial_state;    
%     else
    ode_state = rk4(U_d,Y(j+1,:)'+initial_state);
    initial_state = ode_state(end,:)'-Y(j+2,:)';
%     initial_state = solDV(n+1:2*n);
    x( j + 2, : ) = initial_state;
%     end
    
%     t_discrete = [t(1):0.01:(ceil(t(end)/0.01)*0.01)]'; 
%     u_discrete = zeros(length(t_discrete)-1,2);
%     u_discrete(:,1) = interp1(t(1:end-1),u(:,1),t_discrete(1:end-1),'previous','extrap');
%     u_discrete(:,2) = interp1(t(1:end-1),u(:,2),t_discrete(1:end-1),'previous','extrap');
    
    % simulate in current section
%     U = [U;u_discrete];
%     T = [tspan(1:end-1);time+t_discrete];
%     if(length(t_kept)<=num_del)
%         [x_curr_plus,x_real,y_real] = simu_section(T,U,x0);
%     else
%         t_idx =t_kept(end-num_del);
%         [x_curr_plus,x_real,y_real] = simu_section(T(t_idx:end),...
%             U(t_idx:end,:),x_kept(:,end-num_del));
%     end
%     [x_curr_plus,x_real,y_real] = simu_section(T,U,x0);
%     figure(1)
%     plot(initial_state(1)+Y(j+1,1),initial_state(3)+Y(j+1,3),'r*')%,ode_state(end,1),ode_state(end,3),'*b')
%     drawnow;
%     figure(1)
%     hold on;
%     plot(initial_state(1)+Y(j+1,1),initial_state(3)+Y(j+1,3),'r*');
%     axis([initial_state(1)+Y(j+1,1)-10,initial_state(1)+Y(j+1,1)+10,...
%         initial_state(3)+Y(j+1,3)-10,initial_state(3)+Y(j+1,3)+10])
%     plot(Y(j+1,1),Y(j+1,3),'ob')
%     drawnow
%     figure(2)
%     hold on;
%     plot(j,initial_state(4),'o')
%     plot(j,initial_state(2),'x')
end
