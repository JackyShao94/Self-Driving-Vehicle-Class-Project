function [t,input,x,y,u,v]=ext_result(X,N)
    
    input = [X(1:2:2*N),X(2:2:2*N)];
    t = X((2*N+1):8:end);
    x = X((2*N+2):8:end);
    y = X((2*N+4):8:end);
    u = X((2*N+3):8:end);
    v = X((2*N+5):8:end);
end