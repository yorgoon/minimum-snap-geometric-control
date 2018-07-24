function A = A_const_appA(tau_vec)

m = length(tau_vec);
A = zeros(10*m);
for i = 1:m
    A(10*(i-1)+1:10*i,10*(i-1)+1:10*i) = equalA(tau_vec(i),4);
end

% Continuity constraint
for i = 1:m-1
    A(10*(i-1)+7:10*i,10*i+1:10*(i+1)) = -A(10*(i-1)+12:10*i+5,10*i+1:10*(i+1));
end