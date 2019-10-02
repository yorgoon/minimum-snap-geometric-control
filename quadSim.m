function [tsave, xsave] = quadSim(traj_obj, model_param, KK, t_sim, x0, Fmat, options)



[tsave, xsave] = ode45(@(t,s) quadDynamics(t, s, traj_obj, model_param, KK, Fmat), t_sim, x0, options);

desired_pos = zeros(length(tsave),3);
desired_vel = zeros(length(tsave),3);
desired_acc = zeros(length(tsave),3);

for i = 1:length(tsave)
    desired_s = desiredState(traj_obj, tsave(i));
    desired_pos(i,:) = desired_s.pos';
    desired_vel(i,:) = desired_s.vel';
    desired_acc(i,:) = desired_s.acc';
end

% position,attitude error plot
figure(2)
subplot(4,1,1)
plot(tsave,xsave(:,1)-desired_pos(:,1),'-b','LineWidth',1.0);title('Position error, x-x_d (m)');ylabel('x')
grid on
subplot(4,1,2)
plot(tsave,xsave(:,2)-desired_pos(:,2),'-b','LineWidth',1.0);ylabel('y')
grid on
subplot(4,1,3)
plot(tsave,xsave(:,3)-desired_pos(:,3),'-b','LineWidth',1.0);ylabel('z');%xlabel('time, sec')
grid on
% Orientation error plot (using error function)
yawd = desired_s.yaw;
yawd_dot = desired_s.yawdot;
yawd_2dot = desired_s.yawddot;
xcd = [cos(yawd) sin(yawd) 0]';
zbd = zeros(length(tsave),3);
ybd = zeros(length(tsave),3);
so3_error = zeros(length(tsave),1);
for i = 1:length(tsave)
    des_t = desired_acc(i,:) + [0 0 model_param.grav];
    zbd(i,:) = des_t/norm(des_t);
    ybd(i,:) = (hat_optr(zbd(i,:))*xcd/norm(hat_optr(zbd(i,:))*xcd))';
    xbd = hat_optr(ybd(i,:))*zbd(i,:)';
    Rd = [xbd ybd(i,:)' zbd(i,:)'];
    R = ROTZ(xsave(i,9))*ROTX(xsave(i,7))*ROTY(xsave(i,8));
    so3_error(i) = error_so3(R,Rd);
end
figure(2)
subplot(4,1,4)
plot(tsave,so3_error,'-b','LineWidth',1.0);title('Attitude error function, \Psi');xlabel('time, sec')
grid on
hold off
% Input plot
calc_acc = diff(xsave(:,4:6))./diff(tsave);
calc_ang_acc = diff(xsave(:,10:12))./diff(tsave);
t_vec = calc_acc;
t_vec(:,3) = t_vec(:,3)+model_param.grav;
norm_t = zeros(size(t_vec,1),1);
M = zeros(size(t_vec,1),3);
for i=1:length(norm_t)
    norm_t(i) = norm(t_vec(i,:));
    M(i,:) = model_param.I*calc_ang_acc(i,:)' + hat_optr(xsave(i,10:12))*model_param.I*xsave(i,10:12)';
end
u1 = model_param.mass*norm_t;

% plot for total thrust and moment
% figure(3)
% subplot(4,1,1)
% plot(tsave(1:end-1)+diff(tsave)/2,u1)
% subplot(4,1,2)
% plot(tsave(1:end-1)+diff(tsave)/2,M(:,1))
% subplot(4,1,3)
% plot(tsave(1:end-1)+diff(tsave)/2,M(:,2))
% subplot(4,1,4)
% plot(tsave(1:end-1)+diff(tsave)/2,M(:,3))
U = [u1 M];
L = model_param.arm_length;
c_tf = model_param.c_tf;
mapping_u = [1 1 1 1;0 L 0 -L;-L 0 L 0;c_tf -c_tf c_tf -c_tf];
motor_F = zeros(size(U));
for i=1:length(U)
    trans_F = mapping_u\U(i,:)';
    motor_F(i,:) = trans_F';
end
% plot for thrust of each rotor
figure(3)
subplot(4,1,1)
plot(tsave(1:end-1)+diff(tsave)/2,motor_F(:,1),'-b','LineWidth',1.0);title('Thrust of each rotor, (N)')
grid on
subplot(4,1,2)
plot(tsave(1:end-1)+diff(tsave)/2,motor_F(:,2),'-b','LineWidth',1.0)
grid on
subplot(4,1,3)
plot(tsave(1:end-1)+diff(tsave)/2,motor_F(:,3),'-b','LineWidth',1.0)
grid on
subplot(4,1,4)
plot(tsave(1:end-1)+diff(tsave)/2,motor_F(:,4),'-b','LineWidth',1.0);xlabel('time, sec')
grid on
hold off
end

