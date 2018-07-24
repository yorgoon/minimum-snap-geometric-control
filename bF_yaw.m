function bF_yaw = bF_yaw(initYaw, tau_vec)
%bF_yaw: Construct fixed equality constraint, bF.

% minimze acceleration
r = 2;
% 5th order
n = 2*r+1;
% # of segments
K = length(tau_vec);
% # of knowns (fixed)
m = (n+1)*K - (3*(K-1) + 1);

bF_yaw = zeros(m,1);

bF_yaw(1) = initYaw;