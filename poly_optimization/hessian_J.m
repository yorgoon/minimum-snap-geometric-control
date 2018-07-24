function hessian_J = hessian_J(tau_vec, PATH, gamma, epsilon_t)

K = length(tau_vec);
J = costFunc_appA(tau_vec, PATH, sum(tau_vec), gamma);
hessian_J = zeros(K);

for i = 1:K
    for j = 1:K
        if i ~= j
            tau_vec_ipjp = tau_vec;
            tau_vec_ipjp(i) = tau_vec_ipjp(i) + epsilon_t;
            tau_vec_ipjp(j) = tau_vec_ipjp(j) + epsilon_t;
            Jipjp = costFunc_appA(tau_vec_ipjp, PATH, sum(tau_vec_ipjp), gamma);
            
            tau_vec_ipjm = tau_vec;
            tau_vec_ipjm(i) = tau_vec_ipjm(i) + epsilon_t;
            tau_vec_ipjm(j) = tau_vec_ipjm(j) - epsilon_t;
            Jipjm = costFunc_appA(tau_vec_ipjm, PATH, sum(tau_vec_ipjm), gamma);
            
            tau_vec_imjp = tau_vec;
            tau_vec_imjp(i) = tau_vec_imjp(i) - epsilon_t;
            tau_vec_imjp(j) = tau_vec_imjp(j) + epsilon_t;
            Jimjp = costFunc_appA(tau_vec_imjp, PATH, sum(tau_vec_imjp), gamma);
            
            tau_vec_imjm = tau_vec;
            tau_vec_imjm(i) = tau_vec_imjm(i) - epsilon_t;
            tau_vec_imjm(j) = tau_vec_imjm(j) - epsilon_t;
            Jimjm = costFunc_appA(tau_vec_imjm, PATH, sum(tau_vec_imjm), gamma);
            
            hessian_J(i,j) = (Jipjp-Jipjm-Jimjp+Jimjm)/(4*epsilon_t^2);
        else
            tau_vec_ip = tau_vec;
            tau_vec_ip(i) = tau_vec_ip(i) + epsilon_t;
            Jip = costFunc_appA(tau_vec_ip, PATH, sum(tau_vec_ip), gamma);
            
            tau_vec_im = tau_vec;
            tau_vec_im(i) = tau_vec_im(i) - epsilon_t;
            Jim = costFunc_appA(tau_vec_im, PATH, sum(tau_vec_im), gamma);
            
            hessian_J(i,j) = (-Jim+2*J-Jip)/epsilon_t^2;
        end
        
    end
end

