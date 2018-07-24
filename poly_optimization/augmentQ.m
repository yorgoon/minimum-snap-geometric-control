function QQ = augmentQ(tau_vec,r)
%augmentQ constructs an augmented cost matrix
% nth order poly
n = 2*r+1;
K = length(tau_vec);

QQ = zeros((n+1)*K);
for i = 1:K
    QQ((n+1)*(i-1)+1:(n+1)*i,(n+1)*(i-1)+1:(n+1)*i) = costMat(tau_vec(i),r);
end

