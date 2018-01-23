
% tstep = length(U);
% Y = zeros(tstep+1,6);
% x0=[287,5,-176,0,2,0]';
% Y(1,:) = x0';


% hold on;
% plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'b-','linewidth',1);
% plot(TestTrack.br(1,:),TestTrack.br(2,:),'b-','linewidth',1);
% axis equal
% Y(1:1000,:)=forwardIntegrateControlInput(U(1:10900,:),x0);
% x = Y(i:i+5,1);
% y = Y(i:i+5,3);
% hold on;
% plot(x,y,'b-','linewidth',2);
% drawnow;
% x0 = Y(i+5,:);

% step = 100;
% for i = 1:step:(tstep-step)
%     Y(i+1:i+step,:)=forwardIntegrateControlInput(U(i:i+step-1,:),x0);
%     x = Y(i+1:i+step,1);
%     y = Y(i+1:i+step,3);
%     hold on;
%     plot(x,y,'b-','linewidth',2);
%     drawnow;
%     x0 = Y(i+step,:);
% end

%%
% load test_turn.mat

% U = zeros(5500,2);
% U(:,2) = 0.1;
% U=[0.5*ones(1e4,1),5000*ones(1e4,1)];

figure(2);
tic
Y=forwardIntegrateControlInput(U,x0);
toc

plot(Y(:,1),Y(:,3),'g-','linewidth',1);
hold on;
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'b-','linewidth',1);
plot(TestTrack.br(1,:),TestTrack.br(2,:),'b-','linewidth',1);

T = 0:0.01:(length(U)+1)*0.01;
%%
checkTrajectory([Y(1:end-1,1),Y(1:end-1,3)],U(1:end-1,:))
checkTrajectory([Y(1:end,1),Y(1:end,3)],U(1:end,:))

% [x_curr_plus,x_real,y_real] = simu_section(T,U,x0);
% plot(x_real,y_real,'g-','linewidth',1)
% 
% figure(3);
% Y=forwardIntegrateControlInput_weak(U(1:end,:),x0);
% plot(Y(:,1),Y(:,3),'r-','linewidth',1);
% hold on;
% plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'b-','linewidth',1);
% plot(TestTrack.br(1,:),TestTrack.br(2,:),'b-','linewidth',1);

%%
% tic;u=[0.5*ones(5e4,1),5e3*ones(5e4,1)];[Y]=forwardIntegrateControlInput(u,x0);toc
% %%
