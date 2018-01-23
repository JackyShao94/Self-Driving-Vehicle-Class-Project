function [z0,x,y]=find_feasible(t,u,x0)
% h = 0.01;
h = t(2)-t(1);
tmp_x = x0;
x = zeros(length(t),1);
y = zeros(length(t),1);
x(1) = x0(1);
y(1) = x0(3);
N = length(t)-1;

z0 = zeros(2*N+8*(N+1),1);
u0 = u';
z0(1:2*N) = u0(:);
z0(2*N+1:8:end) = t;
z0(2*N+[2:7]) = x0;
z0(2*N+8:8:end) = t(end);

for i = 1:N
    tmp_t = t(i);
    k1 = dyn_test(tmp_t,tmp_x,u(i,1),u(i,2));
    k2 = dyn_test(tmp_t+h/2,tmp_x+h*k1/2,u(i,1),u(i,2));
    k3 = dyn_test(tmp_t+h/2,tmp_x+h*k2/2,u(i,1),u(i,2));
    k4 = dyn_test(tmp_t+h,tmp_x+h*k3,u(i,1),u(i,2));
    tmp_x = tmp_x + h/6*(k1+2*k2+2*k3+k4);
%     tmp_x = tmp_x + h*k1;
    z0(2*N+8*i+[2:7])=tmp_x;
    x(i+1) =  tmp_x(1);
    y(i+1) =  tmp_x(3);
end

end