%% Constraint C and Ceq
function [c,ceq,dc,dceq] = mycon(z,N,delta_s)
Ndec = length(z);         
c=[];
dc = [];
dceq = sparse(Ndec,8*N);
             
tau0 = z(2*N+1);

ceq=zeros(8*N,1);

for k=1:1:N;
    x=z(2*N+8*(k-1)+(2:7));
    x_plus = z(2*N+8*(k)+(2:7));
    bq = z(2*N+8*(k-1)+(8));
    bq_plus= z(2*N+8*(k)+(8));
    tau= z(2*N+8*(k-1)+(1));
    tau_plus= z(2*N+8*(k)+(1));

    d_f=z(2*k-1);
    F_x=z(2*k);
    
    df = dyn_test(tau,x,d_f,F_x);
    ceq1=tau_plus-(bq-tau0)*delta_s-tau;
    ceq2=x_plus-x-delta_s*(bq-tau0)*df;
    ceq3=-(bq-bq_plus);
    
    ceq(1+8*(k-1):8*k)=[ceq1;ceq2;ceq3];

    dceq1 = sparse(Ndec,1);
    dceq2 = sparse(Ndec,6);
    dceq3 = sparse(Ndec,1);

    dceq1(2*N+8*(k-1)+(1)) = -1; % tau_k
    dceq1(2*N+8*(k)+(1)) = 1; % tau_k+1
    dceq1(2*N+1) = dceq1(2*N+1) + delta_s; % tau_0
    dceq1(2*N+8*(k-1)+(8)) = -delta_s; % b_k

    dceq2(2*N+8*(k-1)+8,:) = -delta_s*df'; % b_k
    dceq2(2*N+1,:) = delta_s*df'; % tau_0
    dceq2(2*N+8*(k-1)+[2:7],:) = -eye(6) - delta_s*(bq-tau0)*dfdx(tau,x,d_f,F_x); % x_k
    dceq2(2*N+8*(k)+[2:7],:) = eye(6); % x_k+1 
    dceq2(2*(k-1)+1,:) = -delta_s*(bq-tau0)*dfdd_f(tau,x,d_f,F_x); % d_f
    dceq2(2*k,:) = -delta_s*(bq-tau0)*dfdFx(tau,x,d_f,F_x); % F_x
    
    dceq3(2*N+8*(k-1)+(8)) = -1; % b_k
    dceq3(2*N+8*(k)+(8)) = 1; % b_k+1

    dceq(:,(1+8*(k-1)):8*k) = [dceq1,dceq2,dceq3];
    % dceq(27,7)
end 
end


