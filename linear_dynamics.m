function [hp] = linear_dynamics(h, R, F, L)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% system parameters
g = L(1);
m = L(2);

%% Angles
R_nc =R;

%% Velocities
v = h(4:6);

%% Ceate vector
hp = zeros(6,1);
e3 = [0;0;1];
hp(1:3) = v;
hp(4:6) = -e3*g + R_nc*e3*(1/m)*F;

end

