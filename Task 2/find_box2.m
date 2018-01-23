function [A, B] = find_box2(Y, Xobs, TestTrack)

N = length(Y);
A = cell(N,1);
B = cell(N,1);

P2_obs = zeros(length(Xobs),2);
for i = 1:length(Xobs)
    Obs_p = cell2mat(Xobs');
    P2_obs(i,:) = Obs_p(4*(i-1)+2,:);
end

% appending the road & trajectory data
cS = [TestTrack.cline,2*TestTrack.cline(:,end)-TestTrack.cline(:,end-1)];
bl = [TestTrack.bl,2*TestTrack.bl(:,end)-TestTrack.bl(:,end-1)];
br = [TestTrack.br,2*TestTrack.br(:,end)-TestTrack.br(:,end-1)];
Y = [Y;2*Y(end,:)-Y(end-1,:)];

Ak=zeros(4,2);
bk= zeros(4,1);

for j = 1:N
    P1 = [Y(j,1),Y(j,3)]; % point j
    P2 = [Y(j+1,1),Y(j+1,3)]; %point j+1, two points from the trajectory
%     P3 = P1-1*(P2-P1);% mirror of j+1 wrt j
%     P2 = P2+1*(P2-P1);

    P3 = P1-2*(P2-P1);% mirror of j+1 wrt j
    P2 = P1+2*(P2-P1);

    % % finding section of P1
    d = TestTrack.bl-repmat(P1',1,length(TestTrack.bl));
    delta_d = sqrt(diag(d'*d));
    [~,idx] = min(delta_d);
    
    T1=bl(:,idx);
    T4=br(:,idx);
    
    DT = T1-T4;
    AS = [DT(2),-DT(1)];
    bS = AS*T1;
    
    if sign(AS*cS(:,idx+1)-bS) == sign(AS*P1'-bS)
        idx = idx;
    else
        idx = idx-1;
    end
    
    
   %  % finding the nearest obstacle
    do = P2_obs'-repmat(P1',1,length(P2_obs));
    delta_do = sqrt(diag(do'*do));  %calculate distance
    [~,i] = min(delta_do);
    
    obst = Xobs{i};
    
    % generate a large box
    T1=bl(:,idx);
    T2=bl(:,idx+1);
    T3=br(:,idx+1);
    T4=br(:,idx);

    DT1 = T1-T2;
    DT3 = T3-T4;
    DP1 = P1-P2;
    DP2 = P1-P3;



    Ak(1,:)=[DT1(2),-DT1(1)];
    Ak(3,:)=[DT3(2),-DT3(1)];
    Ak(2,:)=[-DP1(1), -DP1(2)];
    Ak(4,:)=[-DP2(1), -DP2(2)];

    bk(1)=Ak(1,:)*T1;
    bk(3)=Ak(3,:)*T3;
    bk(2)=Ak(2,:)*P2';
    bk(4)=Ak(4,:)*P3';

    s=sign(Ak*P1'-bk);

    Ak = -s.*Ak;
    bk = -s.*bk;
    
    A{j} = Ak;
    B{j} = bk-A{j}*P1';
    
    
    % check if the obstacle is inside the large box
    
    DP1=obst(1,:)'-obst(2,:)';
    DP2=obst(2,:)'-obst(3,:)';
    DP3=obst(3,:)'-obst(4,:)';
    DP4=obst(4,:)'-obst(1,:)';

    Ao(1,:)=[DP1(2),-DP1(1)];
    Ao(2,:)=[DP2(2),-DP2(1)];
    Ao(3,:)=[DP3(2),-DP3(1)];
    Ao(4,:)=[DP4(2),-DP4(1)];


    Pc = 0.25*(obst(1,:)' + obst(2,:)' + obst(3,:)' + obst(4,:)'); %center point of obstacle

    bo=zeros(4,1);
    bo(1)=Ao(1,:)*obst(1,:)';
    bo(2)=Ao(2,:)*obst(2,:)';
    bo(3)=Ao(3,:)*obst(3,:)';
    bo(4)=Ao(4,:)*obst(4,:)';
    
    sig = sign(Ao*Pc-bo);
    Ao = -sig.*Ao;
    bo = -sig.*bo;
%     flag = isIntersect(Ak,bk,Ao,bo);
    if(all(Ak*Pc<=bk)||isIntersect(Ak,bk,Ao,bo))
        flag = 1;
    else
        flag = 0;
    end
    
%     flag = isInside(obst,P1);
    
    % %
    if flag == 1 % if  obstacle(i) is inside large box
        %smaller box for j

        % going left or right of the obstacle
        T1=bl(:,idx);
        T2=bl(:,idx+1);
        T3=br(:,idx+1);
        T4=br(:,idx);
        
        DT1 = T1-T2;
        DT3 = T3-T4;

    
    
        Ap(1,:)=[DT1(2),-DT1(1)];
        Ap(2,:)=[DT3(2),-DT3(1)];
        
        bp(1)=Ap(1,:)*T1;
        bp(2)=Ap(2,:)*T3;
        % change the distance calculation
        % find which side of the road is obstable at
        dist(1) = abs((Ap(1,:)*obst(1,:)'-bp(1))/norm(Ap(1,:)));
        dist(2) = abs((Ap(2,:)*obst(2,:)'-bp(2))/norm(Ap(2,:)));

        [~,id] = max(dist);
        
        
        AT(1,:) = Ap(id,:);
        bT(1,:) = bp(id);
        
        if id == 1
            idf = 4; %left of obstacle 
        elseif id == 2
            idf = 2; %right of obstacle
        end
        AT(3,:) = Ao(idf,:);
        bT(3,:) = bo(idf,1);
        
        DP1 = P1-P2;
        DP2 = P1-P3;

        AT(2,:)=[-DP1(1), -DP1(2)];
        AT(4,:)=[-DP2(1), -DP2(2)];

        bT(2)= AT(2,:)*P2';
        bT(4)= AT(4,:)*P3';
        
        [LP1]=inv([AT(1,:);AT(2,:)])*[bT(1);bT(2)];
        [LP2]=inv([AT(2,:);AT(3,:)])*[bT(2);bT(3)];
        [LP3]=inv([AT(3,:);AT(4,:)])*[bT(3);bT(4)];
        [LP4]=inv([AT(4,:);AT(1,:)])*[bT(4);bT(1)];

        CP=sum([LP1,LP2,LP3,LP4]')./4; % center of box constraint
            
        s=sign(AT*CP'-bT);
        

        A{j} = -s.*AT;

        B{j} = -s.*bT-A{j}*P1';        
    end
    
%     visual
%     AT = A{j}; bT = B{j}+A{j}*P1';
%         [LP1]=inv([AT(1,:);AT(2,:)])*[bT(1);bT(2)];
%         [LP2]=inv([AT(2,:);AT(3,:)])*[bT(2);bT(3)];
%         [LP3]=inv([AT(3,:);AT(4,:)])*[bT(3);bT(4)];
%         [LP4]=inv([AT(4,:);AT(1,:)])*[bT(4);bT(1)];
%         
%         
%         
%         V = [LP1';LP2';LP3';LP4'];
%     F = [1 2 3 4];
%     
%     figure(2)
%     patch('Faces',F,'Vertices',V,'FaceColor','none')
%     axis equal;
%     pause(0.1)
%     drawnow;
%     axis([P1(1)-10,P1(1)+10,P1(2)-10,P1(2)+10])
end

end
% plot box j
%         AT = A{2}; bT = -s.*bk';
%         [LP1]=inv([AT(1,:);AT(2,:)])*[bT(1);bT(2)];
%         [LP2]=inv([AT(2,:);AT(3,:)])*[bT(2);bT(3)];
%         [LP3]=inv([AT(3,:);AT(4,:)])*[bT(3);bT(4)];
%         [LP4]=inv([AT(4,:);AT(1,:)])*[bT(4);bT(1)];
%         
%         
%         
%         V = [LP1';LP2';LP3';LP4'];
%     F = [1 2 3 4];
%     figure(2)
%     patch('Faces',F,'Vertices',V,'FaceColor','none')
%     axis equal;
%     drawnow;

function flag = isIntersect(Ak,bk,Ao,bo)
    flag = false;
    P1 = [Ak(2,:);Ao(2,:)]\[bk(2,:);bo(2,:)];
    P2 = [Ak(4,:);Ao(2,:)]\[bk(4,:);bo(2,:)];
    P3 = [Ak(2,:);Ao(4,:)]\[bk(2,:);bo(4,:)];
    P4 = [Ak(4,:);Ao(4,:)]\[bk(4,:);bo(4,:)];
    if(all(Ao*P1<=bo)||all(Ao*P2<=bo)||all(Ao*P3<=bo)||all(Ao*P4<=bo))
        flag = true;
    end
%     P = [0;0];
%     fig = figure(1);
%     visual_box(fig,Ak,bk,P);
%     visual_box(fig,Ao,bo,P);
%     plot(P1(1),P1(2),'x');
%     plot(P2(1),P2(2),'x');
%     plot(P3(1),P3(2),'x');
%     plot(P4(1),P4(2),'x');
%     keyboard();
end

