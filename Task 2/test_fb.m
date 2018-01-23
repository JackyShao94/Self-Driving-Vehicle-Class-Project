load test.mat

[Y_ref] = forwardIntegrateControlInput_weak(U_o,x0);

[Y_obs]=forwardIntegrateControlInput_pid(U_o,x0,Y_ref);

figure(1)
plot(Y_obs(:,1),Y_obs(:,3),'--')
hold on
plot(Y_ref(:,1),Y_ref(:,3),'o')

plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'linewidth',1)
plot(TestTrack.br(1,:),TestTrack.br(2,:),'linewidth',1)