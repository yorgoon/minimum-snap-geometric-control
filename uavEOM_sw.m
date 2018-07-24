function sdot = uavEOM_sw(t, s, tau_vec, PATH, P, model_param, K, Fmat)

g = model_param.grav;
m = model_param.mass;
J = model_param.I;

sdot = zeros(12,1);

sd = desired_state(tau_vec, t, PATH, P);

% Thrust
L = model_param.arm_length;
c_tf = model_param.c_tf;
mapping_u = [1 1 1 1;0 L 0 -L;-L 0 L 0;c_tf -c_tf c_tf -c_tf];
[u1, u2, u3, u4] = control_sw2(s, sd, model_param,K);
thrust = mapping_u\[u1, u2, u3, u4]';

epsilon_t = 1e-4;
% 
% while any(abs(thrust)>20)
%     t = t-epsilon_t;
%     sd = desired_state(tau_vec, t, PATH, P);
%     [u1, u2, u3, u4] = control_sw2(s, sd, model_param,K);
%     thrust = mapping_u\[u1, u2, u3, u4]';
% end
F = u1;
M = [u2 u3 u4]';
% % Limit the force and moments due to actuator limits
% A = [0.25,                      0, -0.5/model_param.arm_length;
%      0.25,  0.5/model_param.arm_length,                      0;
%      0.25,                      0,  0.5/model_param.arm_length;
%      0.25, -0.5/model_param.arm_length,                      0];
% 
% prop_thrusts = A*[F;M(1:2)]; % Not using moment about Z-axis for limits
% prop_thrusts_clamped = max(min(prop_thrusts, model_param.maxF/4), model_param.minF/4);
% 
% B = [                 1,                 1,                 1,                  1;
%                       0, model_param.arm_length,                 0, -model_param.arm_length;
%      -model_param.arm_length,                 0, model_param.arm_length,                 0];
% F = B(1,:)*prop_thrusts_clamped;
% M = [B(2:3,:)*prop_thrusts_clamped; M(3)];



R = ROTZ(s(9))*ROTX(s(7))*ROTY(s(8));
omega = [s(10) s(11) s(12)]';

sdot(1) = s(4);
sdot(2) = s(5);
sdot(3) = s(6);

%%%%%%%%%%%%%% Disturbance terms. Must be modified %%%%%%%%%%%%%%
% t_dist_st = 12.5;
% t_dist_end = 14;
% t_dist_st2 = 19;
% t_dist_end2 = 22;
% t_dist_st3 = 32;
% t_dist_end3 = 34;
% 
% if t > t_dist_st 
%     F_dist = [0 0 -6]' * (sin(2*pi/(t_dist_end-t_dist_st)*(t-t_dist_st)-pi/2) + 1);
%     F_dist = [0 0 -2]';
% elseif t > t_dist_st2 && t < t_dist_end2
%     F_dist = [3 0 5]' * (sin(2*pi/(t_dist_end2-t_dist_st2)*(t-t_dist_st2)-pi/2) + 1);
% elseif t > t_dist_st3 && t < t_dist_end3
%     F_dist = [-3 -1.5 0]' * (sin(2*pi/(t_dist_end3-t_dist_st3)*(t-t_dist_st3)-pi/2) + 1);
% else
    F_dist = zeros(3,1);
% end
%%%%%%%%%%%%%% Disturbance terms. Must be modified %%%%%%%%%%%%%%
% F_dist = zeros(3,1);
[~,idx] = min(abs(Fmat(:,1)-t));

zb = R*[0 0 1]';
accel = -g*[0 0 1]' + F/m*zb + Fmat(idx,2:4)'/m;
sdot(4) = accel(1);
sdot(5) = accel(2);
sdot(6) = accel(3);

% relationship between omega & euler angles
mapping_R = [cos(s(8)) 0 -cos(s(7))*sin(s(8));...
             0         1            sin(s(7));...
             sin(s(8)) 0  cos(s(7))*cos(s(8))];
eulerdot = mapping_R\omega;
sdot(7) = eulerdot(1);
sdot(8) = eulerdot(2);
sdot(9) = eulerdot(3);


%M = [u2 u3 u4]';
omegadot = J\(M - hat_optr(omega)*J*omega);
sdot(10) = omegadot(1);
sdot(11) = omegadot(2);
sdot(12) = omegadot(3);

if mod(t,.1) < .0001
    disp(num2str(t))
end

end