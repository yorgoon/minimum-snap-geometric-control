function [Rff,Rfp,Rpf,Rpp] = seg_R_appA(R)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
m = length(R)/10;
unkns = 8*(m-1);
Rff = R(1:end-unkns,1:end-unkns);
Rfp = R(1:end-unkns,end-unkns+1:end);
Rpf = R(end-unkns+1:end,1:end-unkns);
Rpp = R(end-unkns+1:end,end-unkns+1:end);
end

