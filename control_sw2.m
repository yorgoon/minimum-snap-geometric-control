function [u1, u2, u3, u4] = control_sw2(current_state, desired_state, model_param, K)

J = model_param.I; % Moment of inertia wrt body frame

% position error
ep = current_state(1:3)-desired_state.pos;

% velocity error
ev = current_state(4:6)-desired_state.vel;

% desired force, F_des
Kp = K.Kp;
Kv = K.Kv;
KR = K.KR;
K_omega = K.K_omega;
m = model_param.mass;
g = model_param.grav;
Fd = -Kp.*ep -Kv.*ev + m*g*[0 0 1]' + m*desired_state.acc;

% desired input u1
yaw = current_state(9);
roll = current_state(7);
pitch = current_state(8);
R = ROTZ(yaw)*ROTX(roll)*ROTY(pitch); % Current rotation
% current z-axis in body frame
zb = R(:,3);
u1 = Fd'*zb;

current_acc = -g*[0 0 1]'+u1*zb;

% accel error
ea = current_acc-desired_state.acc;
% Desired force derivative
Fd_dot = -Kp*ev -Kv*ea + m*desired_state.jerk;
% desired rotation, Rd = [xbd ybd zbd]
zbd = Fd/norm(Fd);


% ybd = zbd X xcd/norm(zbd X xcd)
% xcd
yawd = desired_state.yaw;
yawd_dot = desired_state.yawdot;
yawd_2dot = desired_state.yawddot;

xcd = [cos(yawd) sin(yawd) 0]';
ybd = hat_optr(zbd)*xcd/norm(hat_optr(zbd)*xcd);
% xbd = ybd X zbd
xbd = hat_optr(ybd)*zbd;
Rd1 = [xbd ybd zbd];
Rd2 = [-xbd -ybd zbd];

% eR orientation error
R1 = (Rd1'*R - R'*Rd1);
R2 = (Rd2'*R - R'*Rd2);

eR1 = 1/2*vee_optr(R1);
eR2 = 1/2*vee_optr(R2);
if norm(eR1) >= norm(eR2)
    Rd = Rd2;
    eR = eR2;
    xbd = -xbd;
    ybd = -ybd;
else
    Rd = Rd1;
    eR = eR1;
end

current_omega = current_state(10:12);

% Desired omega^ = Rd'*R
Fd_norm_dot = Fd'*Fd_dot/norm(Fd);
zbd_dot = (Fd_dot*norm(Fd) - Fd*Fd_norm_dot)/norm(Fd)^2;
xcd_dot = [-sin(yawd) cos(yawd) 0]'*desired_state.yawdot;

zbd_x_xcd_dot = hat_optr(zbd_dot)*xcd + hat_optr(zbd)*xcd_dot;
zbd_xcd_norm_dot = (hat_optr(zbd)*xcd)'*(zbd_x_xcd_dot)/norm(hat_optr(zbd)*xcd);


ybd_dot = (zbd_x_xcd_dot*norm(hat_optr(zbd)*xcd) - hat_optr(zbd)*xcd*zbd_xcd_norm_dot)/norm(hat_optr(zbd)*xcd)^2;
xbd_dot = hat_optr(ybd_dot)*zbd + hat_optr(ybd)*zbd_dot;

Rd_dot = [xbd_dot ybd_dot zbd_dot];
wd_hat = Rd'*Rd_dot;
wd = vee_optr(wd_hat);
% error
ew = current_omega - R'*Rd*wd;

% Desired angular accerelation^ = Rd_dot'*Rd_dot + Rd'*Rd_2dot;
R_dot = R*hat_optr(current_omega);
zb_dot = R_dot(:,3);
u1_dot = Fd_dot'*zb + Fd'*zb_dot;
current_jerk = (u1_dot*zb + u1*zb_dot)/m;
ej = current_jerk - desired_state.jerk;
Fd_2dot = -Kp*ea -Kv*ej + m*desired_state.snap;
zbd_2dot = (Fd_2dot*norm(Fd)-Fd_dot*Fd_norm_dot)/norm(Fd)^2 -...
           (((Fd_dot'*Fd_dot + Fd'*Fd_2dot)*Fd + (Fd'*Fd_dot)*Fd_dot)*norm(Fd)^3 -...
           (Fd'*Fd_dot)*Fd * 3*norm(Fd)*(Fd'*Fd_dot))/norm(Fd)^4;

% ybd_2dot
xcd_2dot = [-cos(yawd)*yawd_dot^2-sin(yawd)*yawd_2dot, -sin(yawd)*yawd_dot^2+cos(yawd)*yawd_2dot, 0]';

ybd_2dot_1 = ((hat_optr(zbd_2dot)*xcd + hat_optr(zbd)*xcd_2dot)*norm(hat_optr(zbd)*xcd)-...
             zbd_x_xcd_dot*zbd_xcd_norm_dot)/norm(hat_optr(zbd)*xcd)^2;

zbd_x_xcd_2dot = hat_optr(zbd_2dot)*xcd + 2*hat_optr(zbd_dot)*xcd_dot + hat_optr(zbd)*xcd_2dot;
zbd_xcd_norm_2dot = ((zbd_x_xcd_dot'*zbd_x_xcd_dot + (hat_optr(zbd)*xcd)'*zbd_x_xcd_2dot)*norm(hat_optr(zbd)*xcd)-...
                    ((hat_optr(zbd)*xcd)'*zbd_x_xcd_dot)*zbd_xcd_norm_dot)/norm(hat_optr(zbd)*xcd)^2;
                
ybd_2dot_2 = ((zbd_x_xcd_dot*zbd_xcd_norm_dot + hat_optr(zbd)*xcd*zbd_xcd_norm_2dot)*norm(hat_optr(zbd)*xcd)^2-...
              2*hat_optr(zbd)*xcd*norm(hat_optr(zbd)*xcd)*zbd_xcd_norm_dot^2)/norm(hat_optr(zbd)*xcd)^4;
          
ybd_2dot = ybd_2dot_1 - ybd_2dot_2;

xbd_2dot = hat_optr(ybd_2dot)*zbd + 2*hat_optr(ybd_dot)*zbd_dot + hat_optr(ybd)*zbd_2dot;

Rd_2dot = [xbd_2dot ybd_2dot zbd_2dot];
wd_dot_hat = Rd_dot'*Rd_dot + Rd'*Rd_2dot;
wd_dot = vee_optr(wd_dot_hat);

% Moment
M = -KR.*eR -K_omega.*ew + hat_optr(current_omega)*J*current_omega -J*(hat_optr(current_omega)*R'*Rd*wd - R'*Rd*wd_dot);

u2 = M(1);u3 = M(2); u4 = M(3);
