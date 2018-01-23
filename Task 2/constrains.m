function [AXcons,bXcons,AUcons,bUcons]=constrains(T,U,Y,TestTrack,Obstacles)
    N_Time = length(T);
    
    AXcons = cell(N_Time,1);
    bXcons = cell(N_Time,1);
    AUcons = cell(N_Time-1,1);
    bUcons = cell(N_Time-1,1);
    
    [A, B] = find_box2(Y, Obstacles, TestTrack);
    
    for i = 1:(N_Time)-1
        Ak = A{i}; Bk = B{i};
        % state constraints
        AXcons{i} = [Ak(:,1),zeros(4,1),Ak(:,2),zeros(4,3);...
                           0,1,zeros(1,4);...
                           0,-1,zeros(1,4);...
                           zeros(2,3),[1;-1],zeros(2,2);...
                           zeros(2,4),[1;-1],zeros(2,1);
                           zeros(2,5),[1;-1]]; % put the bound function here
        bXcons{i} = [Bk;0.2;0.2;0.5;0.5;5;5;5;5]; % put the bound function here
        
        if(i<=N_Time-1)
            % input constraints
            AUcons{i} = [eye(2);-eye(2)];
            bUcons{i} = [0.1;
                      5000-U(i,2);
                      0.1;
                    U(i,2)-(-1e5)];
%                 bUcons{i} = [0.05;
%                       100;
%                       0.05;
%                     100];
        end
    end

end