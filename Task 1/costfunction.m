%% Cost function
function [cost,gradient]=costfunction(z,Ndec);
    cost = z(end);
    gradient=zeros(1,Ndec)';
    gradient(end)=1;
end