function bF = const_bF_appA(PATH)

% number of segments
m = length(PATH)-1;
b = zeros(10*m,1);

% Start
b(1) = PATH(1);
% end
b(end-4) = PATH(end);
% waypoint positions
for i = 1:m-1
    b(10*(i-1)+6) = PATH(i+1);
    b(10*(i-1)+11) = PATH(i+1);
end

% Permutation
C = permut_mat_appA(m);
b_sorted = C*b;
% number of unknowns
unkns = 8*(m-1);
bF = b_sorted(1:end-unkns);
