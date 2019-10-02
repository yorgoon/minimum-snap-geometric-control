classdef MinimumSnapTrajectory < handle
   properties
       tau_vec
       path
       A
       C
       Q
       P
       J
   end
   methods
      % Constructor
      function obj = MinimumSnapTrajectory(tau_vec, path)
         if nargin == 2
            if isnumeric(tau_vec) && isvector(tau_vec)
               obj.tau_vec = tau_vec;
            else
               error('tau_vec is either not numeric or not a column vector')
            end
            if isnumeric(path) && size(path, 1) == length(tau_vec)+1
                obj.path = path;
            else
                error('path is either not numeric or the size is incorrect')
            end
            bF = fixedB(obj);
            obj.A = augmentA(obj);
            obj.C = permutMat(obj);
            obj.Q = augmentQ(obj);
            b_sorted = sortedB(obj, bF);
            obj.P = (obj.C*obj.A)\b_sorted;
            obj.J = trace(obj.P'*obj.Q*obj.P);
         end
      end
      
      function A = equalA(~, tau, r)
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
      end
      
      % Augment A
      function augA = augmentA(obj)
        m = length(obj.tau_vec);
        augA = zeros(10*m);
        for i = 1:m
            augA(10*(i-1)+1:10*i,10*(i-1)+1:10*i) = equalA(obj, obj.tau_vec(i),4);
        end
        % Continuity constraint
        for i = 1:m-1
            augA(10*(i-1)+7:10*i,10*i+1:10*(i+1)) = -augA(10*(i-1)+12:10*i+5,10*i+1:10*(i+1));
        end
      end
      
      % Permutation C
      function C = permutMat(obj)
        m = length(obj.tau_vec);
        C = eye(10*m);
        c = [];
        idx = [];
        for i=1:m-1
            c = [c;C(10*i+2:10*i+5,:)];
            idx = [idx, 10*i+2:10*i+5];
        end
        C(idx,:) = [];
        C = [C;c];
      end
      
      % Fixed b
      function bf = fixedB(obj)
        PATH = obj.path;
        m = size(PATH,1);
        bf = zeros(10*(m-1),size(PATH,2));
        bf(1,:) = PATH(1,:);
        bf(end-4,:) = PATH(end,:);
        idx = 1;
        for i=1:m-2
            bf(5*(idx)+1,:) = PATH(i+1,:);
            idx = idx+1;
            bf(5*(idx)+1,:) = PATH(i+1,:);
            idx = idx+1;
        end
        idx = [];
        for i=1:m-2
            idx = [idx, 10*i+2:10*i+5];
        end
        bf(idx,:) = [];
      end
      
      % Cost Q
      function Q = costMat(obj, tau, r)
        n = 2*r+1;
        Q = zeros(n+1);
        QQ = zeros(n+1,n+1,length(r));
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
      end
      
      % Augment Q
      function augQ = augmentQ(obj)
        %augmentQ constructs an augmented cost matrix
        r = 4;
%         Tau_vec = obj.tau_vec;
        n = 2*r+1;
        m = length(obj.tau_vec);
        augQ = zeros((n+1)*m);
        for i = 1:m
            augQ((n+1)*(i-1)+1:(n+1)*i,(n+1)*(i-1)+1:(n+1)*i) = costMat(obj, obj.tau_vec(i), r);
        end
      end
      
      function [Rff,Rfp,Rpf,Rpp] = seg_R(obj, R)
        m = length(R)/10;
        unkns = 4*(m-1);
        Rff = R(1:end-unkns,1:end-unkns);
        Rfp = R(1:end-unkns,end-unkns+1:end);
        Rpf = R(end-unkns+1:end,1:end-unkns);
        Rpp = R(end-unkns+1:end,end-unkns+1:end);
      end
        
      % Optimized b
      function b_sorted = sortedB(obj, bF)
        R = obj.C/obj.A'*obj.Q/obj.A*obj.C';
        [~,Rfp,~,Rpp] = seg_R(obj, R);
        % Optimized bP
        bP_opt = -Rpp\Rfp'*bF;
        b_sorted = [bF;bP_opt];
      end
   end
end
