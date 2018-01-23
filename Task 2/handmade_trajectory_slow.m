clc;close all;clear all;

load('TestTrack');
% Nobs = 30;
% Xobs = generateRandomObstacles(Nobs,TestTrack);


% load very_slow_trajectory

% Visualization
figure(1);
plot(TestTrack.cline(1,:),TestTrack.cline(2,:),'linewidth',1);
hold on;
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'linewidth',1)
plot(TestTrack.br(1,:),TestTrack.br(2,:),'linewidth',1)
hold on
% for i = 1:length(Xobs)
%     V = Xobs{i};
%     F = [1 2 3 4];
%     patch('Faces',F,'Vertices',V,'FaceColor','none')
%     axis equal;
% end
theta = TestTrack.theta;

x0=[287,5,-176,0,2,0]';

% first straight
d = -0.0055;
d_f = linspace(d,0,2760)';
F_x = 80*ones(2760,1);

U = [d_f,F_x];

% first turn

d = -0.026;
d_f = linspace(0,d,5)';
F_x = linspace(80,70,5)';

U = [U;d_f,F_x];

d_f = d*ones(2600,1);
F_x = 70*ones(2600,1);

U = [U;d_f,F_x];

% % second straight

d_f = linspace(d,0,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = 0*ones(3000,1);
F_x = 70*ones(3000,1);

U = [U;d_f,F_x];

% % % left turn

d = 0.031;
d_f = linspace(0,d,5)';
F_x = linspace(70,70,5)';

U = [U;d_f,F_x];

d_f = d*ones(1100,1);
F_x = 70*ones(1100,1);

U = [U;d_f,F_x];

% % second straight

d_f = linspace(d,0,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = 0*ones(480,1);
F_x = 70*ones(480,1);

U = [U;d_f,F_x];

% % % right turn

d = -0.035;
d_f = linspace(0,d,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = d*ones(1300,1);
F_x = 70*ones(1300,1);

U = [U;d_f,F_x];

% % % third straight

d_f = linspace(d,0,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = 0*ones(300,1);
F_x = 70*ones(300,1);

U = [U;d_f,F_x];

% % % left turn

d = 0.045;
d_f = linspace(0,d,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = d*ones(1250,1);
F_x = 70*ones(1250,1);

U = [U;d_f,F_x];

% % % forth straight

d_f = linspace(d,0,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = 0*ones(600,1);
F_x = 70*ones(600,1);

U = [U;d_f,F_x];

% % % right turn

d = -0.0285;
d_f = linspace(0,d,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = d*ones(1600,1);
F_x = 70*ones(1600,1);

U = [U;d_f,F_x];

% % % forth straight

d_f = linspace(d,0,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = 0*ones(400,1);
F_x = 70*ones(400,1);

U = [U;d_f,F_x];

% % % right turn

d = -0.035;
d_f = linspace(0,d,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = d*ones(1400,1);
F_x = 70*ones(1400,1);

U = [U;d_f,F_x];

% % % forth straight

d_f = linspace(d,0,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = 0*ones(1780,1);
F_x = 70*ones(1780,1);

U = [U;d_f,F_x];

% % % left turn

d = 0.11;
d_f = linspace(0,d,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = d*ones(450,1);
F_x = 70*ones(450,1);

U = [U;d_f,F_x];

% % % forth straight

d_f = linspace(d,0.0115,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = 0.0115*ones(1900,1);
F_x = 70*ones(1900,1);

U = [U;d_f,F_x];
 
d_f = linspace(0.0115,0,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

% % % right turn
d = -0.051;

d_f = linspace(0,d,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = d*ones(2190,1);
F_x = 70*ones(2190,1);

U = [U;d_f,F_x];

% % % left turn

d = 0.11;
d_f = linspace(-0.051,d,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = d*ones(675,1);
F_x = 70*ones(675,1);

U = [U;d_f,F_x];

% % % forth straight

d_f = linspace(d,0,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = 0*ones(3280,1);
F_x = 70*ones(3280,1);

U = [U;d_f,F_x];

% % % left turn

d = 0.1;
d_f = linspace(0,d,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = d*ones(345,1);
F_x = 70*ones(345,1);

U = [U;d_f,F_x];

% % % forth straight

d_f = linspace(d,0,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = 0*ones(5500,1);
F_x = 70*ones(5500,1);

U = [U;d_f,F_x];
% 
d = 0.005;
d_f = linspace(0,d,5)';
F_x = 70*ones(5,1);

U = [U;d_f,F_x];

d_f = d*ones(2000,1);
F_x = 70*ones(2000,1);

U = [U;d_f,F_x];

[Y]=forwardIntegrateControlInput(U,x0);

figure
plot(U(:,1))
figure
plot(U(:,2))
figure(1)
plot(Y(:,1),Y(:,3),'--')
hold on
xlim([Y(end,1)-50,Y(end,1)+50])
ylim([Y(end,3)-50,Y(end,3)+50])

Y(end,2)