function J_grad = costJacobian(traj_obj, gamma, epsilon_t)
tau_vec = traj_obj.tau_vec;
path = traj_obj.path;

epsilon_t = [epsilon_t, -epsilon_t];
J_pur = zeros(length(tau_vec),2);
for n = 1:2
    for i = 1:length(tau_vec)
        tau_vec_pur = tau_vec;
        tau_vec_pur(i) = tau_vec_pur(i) + epsilon_t(n);
        J_pur(i,n) = computeCost(MinimumSnapTrajectory(tau_vec_pur, path), gamma);
    end
end
J_grad = (J_pur(:,1)-J_pur(:,2))./(2*epsilon_t(1));
J_grad = J_grad';

end

