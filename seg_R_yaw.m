function [Rff,Rfp,Rpf,Rpp] = seg_R_yaw(R)
%seg_R_yaw

% minimze acceleration
r = 2;
% 5th order
n = 2*r+1;
% # of segments
K = length(R)/(n+1);
% # of knowns (fixed)
m = (n+1)*K - (3*(K-1) + 1);

Rff = R(1:m, 1:m);
Rfp = R(1:m, m+1:end);
Rpf = R(m+1:end, 1:m);
Rpp = R(m+1:end, m+1:end);
end
