function [hp] = linear_dynamics2D(h, F, L)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% system parameters
g = L(1);
m = L(2);
Ixx= L(3);


psi = h(3);

%% Angles
R = [(-1/m)*sin(psi) 0;
    (1/m)*cos(psi) 0;
    0 1/Ixx];

%% Velocities
v = h(4:6);

%% Ceate vector
hp = zeros(6,1);
e2 = [0;1;0];
hp(1:3) = v;
hp(4:6) = -e2*g + R*F;

end

