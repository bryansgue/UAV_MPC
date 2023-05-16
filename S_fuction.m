function [S] = S_fuction(X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


S = zeros(6,6);
S(3,3) = X(5);
S(4,4) = X(6);
S(5,5) = X(7);
S(6,6) = X(8);

end

