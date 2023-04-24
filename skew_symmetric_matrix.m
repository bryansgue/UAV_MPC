function [S] = skew_symmetric_matrix(a)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
a1 = a(1);
a2 = a(2);
a3 = a(3);


S = [0 -a3 a2;...
     a3 0 -a1;...
     -a2 a1 0];
end

