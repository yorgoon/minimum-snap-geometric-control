function Q = costMat(tau, r)
%costMat constructs a cost matrix, Q for a single segment
% r: The order of the derivative subjected to optimization
% ex) r = 4: minimum snap, r = 2: minimum acceleration

% nth order poly
n = 2*r+1;

Q = zeros(n+1);
QQ = zeros(n+1,n+1,length(r));
% eg optimizing 4th derivative
% r = 4;
% row
for rr = 1:length(r)
    for i=0:size(Q,1)-1
        % column
        for j=0:size(Q,2)-1
            if i >= r(rr) && j >= r(rr)
                m = 0:r(rr)-1;
                QQ(i+1,j+1,rr) = 2*prod((i-m).*(j-m))*tau^(i+j-2*r(rr)+1)/(i+j-2*r(rr)+1);
            end
        end
    end
end

for i = 1:length(r)
    Q = Q + QQ(:,:,i);
end