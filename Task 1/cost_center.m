%% Cost function
function [cost,gradient]=cost_center(z,Ndec,Pc_plus,theta_list)
    x = z(end-8+2);
    y = z(end-8+4);
    theta = theta_list(3);
    theta0 = theta_list(2); % next line
    n = [cos(theta0);sin(theta0)]; % direction of track
    P = eye(2) - n*n'; % projection matrix
    x_tilde = [x;y]-Pc_plus;
    
    psi = z(end-8+1+5);  %heading
    v = z(end-8+3);      %longitudnal speed
    
    c0 = 20; 
    weight = exp(0.05*(1:length(theta_list)))';
    weight = weight/sum(weight);
    c2 = (exp(5*abs(theta_list-theta_list(1)))-1)'*weight*0.01;
    c1 = 20/norm(x_tilde)^2;%0.5/(c2)/norm(x_tilde)^2;
    if(c2<=0.001)
        c1 = 50/norm(x_tilde)^2;
    end
    c3 = 20;
    
    cost = c0*z(end) + c1*x_tilde'*P*x_tilde + c2*(v-8)^2 + c3*(psi-theta)^2;
    gradient=zeros(1,Ndec)';
    gradient(end)=c0;
%     gradient(end-8+2) = 2*(x-Pc_plus(1));
%     gradient(end-8+4) = 2*(y-Pc_plus(2));
    gradient(end-8+[2;4]) = 2*c1*P*x_tilde;
    gradient(end-8+3) = 2*c2*(v-8);
    gradient(end-8+1+5) = 2*c3*(psi-theta);
end