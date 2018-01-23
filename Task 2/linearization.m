function [A,B] = linearization(T,U,Y)
    N_Time = length(T);
    
    A = cell(N_Time,1);
    B = cell(N_Time,1);
    
    for i = 1:(N_Time-1)
        A{i} = dfdx(T(i),Y(i,:)',U(i,1),U(i,2))'; % transpose is required
        B{i} = [dfdd_f(T(i),Y(i,:)',U(i,1),U(i,2))',...
            dfdFx(T(i),Y(i,:)',U(i,1),U(i,2))'];
        
    end
    
end