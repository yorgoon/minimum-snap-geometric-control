function A = equalA(tau, r)
%equalA constructs a matrix which maps the coefficients of the polynomial to 
% equality constraints
% Ax = b
% r: The order of derivative subjected to optimization
% Full control from 0th to rth derivatives => nth order poly = (2r+1)th

% nth order order poly
n = 2*r+1;
A0 = zeros(r+1,n+1);
Atau = zeros(r+1,n+1);

for i = 0:r
    for j = 0:n
        if j == i
            m = 0:i-1;
            A0(i+1,j+1) = prod(j-m);
        end
        if j >= i
            m = 0:i-1;
            Atau(i+1,j+1) = prod(j-m)*tau^(j-i);
        end
    end
end

A = [A0;Atau];
