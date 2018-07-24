function desired_state = desired_state(tau_vec, t, PATH, P)

total_time = sum(tau_vec);
ts = [0; cumsum(tau_vec)];
p = PATH;
if t >= total_time
    pos = p(end,:);
    vel = [0;0;0];
    acc = [0;0;0];
    jerk = [0;0;0];
    snap = [0;0;0];
else
    k = find(ts<=t);
    k = k(end);
    
    pos = zeros(1,3);
    vel = zeros(1,3);
    acc = zeros(1,3);
    jerk = zeros(1,3);
    snap = zeros(1,3);
    for m = 1:size(p,2)
        tau = t-ts(k);
        pos(m) = [tau^9,tau^8,tau^7,tau^6,tau^5,tau^4,tau^3,tau^2,tau,1] * P(10*k:-1:10*(k-1)+1,m);
        vel(m) = [9*tau^8,8*tau^7,7*tau^6,6*tau^5,5*tau^4,4*tau^3,3*tau^2,2*tau,1,0] * P(10*k:-1:10*(k-1)+1,m);
        acc(m) = [72*tau^7,56*tau^6,42*tau^5,30*tau^4,20*tau^3,12*tau^2,6*tau,2,0,0] * P(10*k:-1:10*(k-1)+1,m);
        jerk(m) = [504*tau^6,336*tau^5,210*tau^4,120*tau^3,60*tau^2,24*tau,6,0,0,0] * P(10*k:-1:10*(k-1)+1,m);
        snap(m) = [3024*tau^5,1680*tau^4,840*tau^3,360*tau^2,120*tau,24,0,0,0,0] * P(10*k:-1:10*(k-1)+1,m);
%         pos(m) = polyval(P(10*k:-1:10*(k-1)+1,m),t-ts(k));
%         vel(m) = polyval(polyder(P(10*k:-1:10*(k-1)+1,m)),t-ts(k));
%         acc(m) = polyval(polyder(polyder(P(10*k:-1:10*(k-1)+1,m))),t-ts(k));
%         jerk(m) = polyval(polyder(polyder(polyder(P(10*k:-1:10*(k-1)+1,m)))),t-ts(k));
%         snap(m) = polyval(polyder(polyder(polyder(polyder(P(10*k:-1:10*(k-1)+1,m))))),t-ts(k));
    end
end

yaw = 0;
yawdot = 0;
yawddot = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.jerk = jerk(:);
desired_state.snap = snap(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
desired_state.yawddot = yawddot;