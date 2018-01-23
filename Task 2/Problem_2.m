clc;close all;clear all;
tic

load('TestTrack');
Nobs = 10;
% Xobs = generateRandomObstacles(Nobs,TestTrack);
load xobs-1

load handmade_trajectory_slow

%%
% Visualization
figure(1);
plot(Y(:,1),Y(:,3),'-');
hold on;
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'linewidth',1)
plot(TestTrack.br(1,:),TestTrack.br(2,:),'linewidth',1)

for i = 1:length(Xobs)
    V = Xobs{i};
    F = [1 2 3 4];
    patch('Faces',F,'Vertices',V,'FaceColor','none')
    axis equal;
end
%%
Xobs_vir = Xobs;
ratio_large = 1;
for i = 1:length(Xobs)
    P_center = mean(Xobs{i});
    Xobs_vir{i} = ratio_large*(Xobs{i} - repmat(P_center,4,1)) + repmat(P_center,4,1);
end

x0 = [287,5,-176,0,2,0]';
dx0 = zeros(6,1);
T = 0:0.01:(size(U,1))*0.01;
 
N_Time = length(T);
q = 20;
for i = 1:floor(N_Time/q)
    Yn(i,:) = Y((i-1)*q+1,:);
    Tn(i) = T((i-1)*q+1);
    Un(i,:) = U((i-1)*q+1,:);
end

PredHorizon = 40;
% Un = [Un;Un(end-PredHorizon-1:end,:)];
% Yn = [Yn;Yn(end-PredHorizon-1:end,:)];
% Tn = 


% Ndec = 6*PredHorizon+2*(PredHorizon-1);

[A,B] = linearization(Tn,Un,Yn);
[AXcons,bXcons,AUcons,bUcons]=constrains(Tn,Un,Yn,TestTrack,Xobs_vir);


    
Q = [100 0 0 0 0 0;...
     0 0 0 0 0 0;...
     0 0 1 0 0 0;...
     0 0 0 0 0 0;...
     0 0 0 0 0 0;...
     0 0 0 0 0 0];
% Q = eye(6);
R = [0 0;0 0];
    
[x, U_o] = mpc_LTV( A, B, Q, R, Tn, PredHorizon, AXcons,...
        bXcons, AUcons, bUcons, dx0, Yn , U, q);
    
%%

U_t = [U_o;U(length(U_o)+1:end,1),U(length(U_o)+1:end,2)];


[Y_obs]=forwardIntegrateControlInput(U_t,x0);

figure(1)
plot(Y_obs(:,1),Y_obs(:,3),'--')
hold on

toc


%%
log = checkTrajectory(Y_obs,U_t,Xobs,TestTrack)

% [Y_obs]=forwardIntegrateControlInput_one(Unt,x0, Tn(2)-Tn(1));
% 
% figure(1)
% plot(Y_obs(:,1),Y_obs(:,3),'--')
% hold on