function b_sorted = b_srtd_yaw(R_yaw, bFyaw)

[~,Rfp,~,Rpp] = seg_R_yaw(R_yaw);

% Optimized bP
bP_opt = -Rpp\Rfp'*bFyaw;
b_sorted = [bFyaw;bP_opt];


end
