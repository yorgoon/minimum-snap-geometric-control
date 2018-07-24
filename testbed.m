clear
clc
PATH = rand(3,3);
tau_vec = rand(size(PATH,1)-1,1)*2+1;
t = rand(10,1)*sum(tau_vec);
%t = 0;
K = length(tau_vec);
% Mapping matrix from p to equality constraint, b
A = A_const_appA(tau_vec);
A_yaw = augmentA_yaw(tau_vec);
% Permutation matrix to rearrange b
C = permut_mat(K);
C_yaw = permutMat_yaw(tau_vec);
% Cost matrix
Q = augmentQ(tau_vec,4);
Q_yaw = augmentQ(tau_vec,2);
% C*A^(-T)*Q*A^(-1)*C^(T) = R
R = comp_R_appA(A,C,Q);
R_yaw = comp_R_appA(A_yaw,C_yaw,Q_yaw);
% Known derivatives including position. 
bF = const_bf(PATH);
bFyaw = bF_yaw(0, tau_vec);
% Unknown b terms are optimized.
b_sorted = comp_b_sorted(R, bF);
b_srtdYaw = b_srtd_yaw(R_yaw, bFyaw);
% C*A*P = b_sorted
% P = [p0 p1 ... p8 p9| ... ] where P(t) = p9t^9 + p8t^8 + ... + p0
P = (C*A)\b_sorted;
states_matlab = [];
for i=1:length(t)
    des_s = desired_state(tau_vec, t(i), PATH, P);
    ParticleState = [des_s.pos des_s.vel des_s.acc des_s.jerk]';
    Snap = des_s.snap';
    states_matlab = [states_matlab; ParticleState;Snap];
end

dlmwrite('t_matlab.csv',t, 'precision', '%2.16f')
dlmwrite('path_matlab.csv',PATH, 'precision', '%2.16f')
dlmwrite('tau_vec_matlab.csv',tau_vec, 'precision', '%2.16f')
dlmwrite('states_matlab.csv',states_matlab, 'precision', '%2.16f')
