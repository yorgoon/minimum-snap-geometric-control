function desired_state = desiredState(traj_obj, t)
%desiredState computes desired position (x,y,z), yaw and its derivatives 
% INPUT
% 

tau_vec = traj_obj.tau_vec';
P = traj_obj.P;
path = traj_obj.path;

% Check dimensions
if size(path,2) ~= size(P,2)
    path = path';
end

cumsum_tau_vec = cumsum(tau_vec);
ts = [0; cumsum_tau_vec(:)];
D = size(P,2);

% Declare variables
pos = zeros(1,D);
vel = zeros(1,D);
acc = zeros(1,D);
jerk = zeros(1,D);
snap = zeros(1,D);


% If time is out of range, return the end point of the path
if t < 0
    error('time has to be greater than zero.')
elseif t >= sum(tau_vec)
    pos = path(end,:);
else
    k = find(ts<=t);
    k = k(end);
    for m = 1:D
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

% Desired yaw and its derivatives are all zero.
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