function [R_p] = angular_dynamics_r(R,omega)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

a1 = omega(1);
a2 = omega(2);
a3 = omega(3);


S = [0 -a3 a2;...
     a3 0 -a1;...
     -a2 a1 0];

R_p = R*S;
end

