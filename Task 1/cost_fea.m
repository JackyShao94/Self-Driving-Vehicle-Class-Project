%% Cost function
function [cost,gradient]=cost_fea(z,Ndec);
    cost = 1;
    gradient=zeros(1,Ndec)';
    gradient(end)=0;
end