function b_sorted = comp_b_sorted(R, bF)

[~,Rfp,~,Rpp] = seg_R(R);

% Optimized bP
bP_opt = -Rpp\Rfp'*bF;
b_sorted = [bF;bP_opt];


end

