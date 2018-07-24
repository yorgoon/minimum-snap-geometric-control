function C = permutMat_yaw(tau_vec)
%permutMat_yaw takes tau_vec as an input and produces permutation matrix for
%yaw. Only initial conditions (yaw, yawdot, yaw2dot) and final (yawdot,
%yaw2dot) is being specified. Rest will be optimized with equality
%constraints.
r = 2; % up to 2nd derivative
n = 2*r+1; % 5th order
K = length(tau_vec);
C = eye((n+1)*K);
c = [];
idx = [];
for i=1:K-1
    if i == K-1
        ukn_row = (n+1)*i+1:(n+1)*i+4;  % Last term includes last angle
    else
        ukn_row = (n+1)*i+1:(n+1)*i+3;  % unknown rows: 7,8,9 , 13,14,15, ...
    end
    c = [c;C(ukn_row,:)];
    idx = [idx, ukn_row];
end
C(idx,:) = [];
C = [C;c];

