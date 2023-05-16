function [Q] = Q_fuction(X)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


Q = zeros(6,6);
Q(4,4) = X(9);
Q(5,5) = X(10);
Q(6,6) = X(11);

end

