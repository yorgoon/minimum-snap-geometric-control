function J = cost_func2(PATH,del_ts,gamma)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
ts = [0 cumsum(del_ts)];
A = A_const(ts);
%A = A + eye(size(A))*eps;
C = permut_mat(ts);
Q = cost_mat(ts);
%
R = C/A'*Q/A*C';
%
[Rff,Rfp,Rpf,Rpp] = seg_R(R);
bf = const_bf(PATH);
bp_star = -Rpp\Rfp'*bf;
b_srtd = [bf;bp_star];

% cost
J = b_srtd'*R*b_srtd + gamma*ts(end);

end

