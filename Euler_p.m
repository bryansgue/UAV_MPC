function [euler_p] = Euler_p(omega,euler)
%UNTITLED2 Summary of this function goes here

%% Euler values
phi = Angulo(euler(1));
theta = Angulo(euler(2));
psi = Angulo(euler(3));



W = [1 sin(euler(1))*tan(euler(2)) cos(euler(1))*tan(euler(2));...
    0 cos(euler(1)) sin(euler(1));...
    0 sin(euler(1))/cos(euler(2)) cos(euler(1))/cos(euler(2))];

euler_p = W*omega;
end

