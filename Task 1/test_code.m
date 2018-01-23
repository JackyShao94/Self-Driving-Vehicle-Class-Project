% 
% x = 10000+zeros(6,1);
% t = 0;
% d_f= 3.2000e-04;
% F_x=100;

ddf = dfdd_f(t,x,d_f,F_x)'

% gradient(@(x) dfdx(t,x,d_f,F_x),x)
deta = 1e-4;
(dyn_test(t,x,d_f+deta,F_x)-dyn_test(t,x,d_f-deta,F_x))/deta/2


tmp_df = d_f+[-0.1:1e-5:0.1];
for i = 1:length(tmp_df)
    Z(:,i) = dyn_test(t,x,tmp_df(i),F_x);
    dZ(:,i) = dfdd_f(t,x,tmp_df(i),F_x)';
    dZ2(:,i)=(dyn_test(t,x,tmp_df(i)+deta,F_x)-dyn_test(t,x,tmp_df(i)-deta,F_x))/deta/2;
end
figure(1);
subplot(2,1,1)
plot(tmp_df,Z(4,:))
subplot(2,1,2);
plot(tmp_df,dZ(4,:))
% hold on;
% plot(tmp_df,dZ2(4,:))

axis('auto')

figure(2);
subplot(2,1,1)
plot(tmp_df,Z(6,:))
subplot(2,1,2);
plot(tmp_df,dZ(6,:))
% hold on;
% plot(tmp_df,dZ2(6,:))

axis('auto')
