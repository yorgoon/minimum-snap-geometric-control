function J_total = J_total(Q,X,ts,gamma)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
m = length(ts) - 1;
   QQ = zeros(10*m,10*m);

   for k=1:m
       T1 = zeros(10);
       T1(1:6,1:6) = comp_T(ts(k));
       T2 = zeros(10);
       T2(1:6,1:6) = comp_T(ts(k+1));
       T = T2-T1;
       QQ(10*(k-1)+1:10*k,10*(k-1)+1:10*k) = Q.*T;
   end
    J_total = X'*QQ*X + gamma*ts(end);
end

