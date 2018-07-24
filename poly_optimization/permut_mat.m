function C = permut_mat(K)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%m = length(ts);
C = eye(10*K);
c = [];
idx = [];
for i=1:K-1
    c = [c;C(10*i+2:10*i+5,:)];
    idx = [idx, 10*i+2:10*i+5];
end
C(idx,:) = [];
C = [C;c];

