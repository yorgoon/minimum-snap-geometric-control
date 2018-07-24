function C = permut_mat_appA(K)

% K = number of segments
m = K;
C = eye(10*m);
idx = [];
for i = 1:m-1
    % unknown indices
    idx = [idx, 7 + 10*(i-1) : 10 + 10*(i-1), 7 + 10*(i-1)+5 : 10 + 10*(i-1)+5];
end

C_unkns = C(idx,:);
C(idx,:) = [];
C = [C;C_unkns];