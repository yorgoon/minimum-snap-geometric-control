function A = augmentA_yaw(tau_vec)

% Minimum acceleration = 5th order polynomial
r = 2;
n = 2*r+1;
m = length(tau_vec);
A = zeros((n+1)*m);
for i = 1:m
    A((n+1)*(i-1)+1:(n+1)*i,(n+1)*(i-1)+1:(n+1)*i) = equalA(tau_vec(i),r);
end

% Continuity constraint
% For yaw, we only specify starting angle
for i = 1:m-1
    A((n+1)*(i-1)+(n+1)/2+1:(n+1)*i,(n+1)*i+1:(n+1)*(i+1)) = ...
   -A((n+1)*(i-1)+(n+1)+1:(n+1)*i+(n+1)/2,(n+1)*i+1:(n+1)*(i+1));
end