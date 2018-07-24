function [A,C,b_srtd,J] = cost_func(PATH,ts,gamma)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
A = A_const(ts);
Ae = eig(A);
  if (min(Ae) < 0)
    % apply a trust-region like Hessian modification
    A = A + eye(size(A))*(abs(min(Ae)) + .001);
  end
%A = A + eye(size(A))*eps;
C = permut_mat(ts);
Q = cost_mat(ts);
%
R = C/A'*Q/A*C';
Re = eig(R);
  if (min(Re) < 0)
    % apply a trust-region like Hessian modification
    R = R + eye(size(R))*(abs(min(Re)) + .001);
  end
%
J = 0;
b_srtd = [];
[Rff,Rfp,Rpf,Rpp] = seg_R(R);
for i=1:size(PATH,2)
    bf = const_bf(PATH(:,i));
    bp_star = -Rpp\Rfp'*bf;
    b_srtd(:,i) = [bf;bp_star];
    % cost
    J = J + b_srtd(:,i)'*R*b_srtd(:,i) + gamma*ts(end);
end

end

