function [x0_plus,x,y]=simu_section(t,u,x0)

x = zeros(length(u),1);
y = zeros(length(u),1);
% x(1) = x0(1);
% y(1) = x0(3);
% 
% num_divide = 10;
% h = 0.01/num_divide;
% % h = t(2)-t(1);
% tmp_x = x0;
% 
% for i = 1:num_divide*(length(u)-1)
%     idx = floor(i/num_divide)+1;
%     tmp_t = t(idx)+h*(mod(i+2,num_divide));
%     k1 = dyn_test(tmp_t,tmp_x,u(idx,1),u(idx,2));
%     k2 = dyn_test(tmp_t+h/2,tmp_x+h*k1/2,u(idx,1),u(idx,2));
%     k3 = dyn_test(tmp_t+h/2,tmp_x+h*k2/2,u(idx,1),u(idx,2));
%     k4 = dyn_test(tmp_t+h,tmp_x+h*k3,u(idx,1),u(idx,2));
%     tmp_x = tmp_x + h/6*(k1+2*k2+2*k3+k4);
% %     tmp_x = tmp_x + h*k1;
%     x(i+1) =  tmp_x(1);
%     y(i+1) =  tmp_x(3);
% end
% x0_plus = tmp_x;


[Y]=forwardIntegrateControlInput_weak(u,x0);
x = Y(:,1);
y = Y(:,3);
x0_plus = Y(end,:)';
end