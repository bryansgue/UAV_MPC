function [angles_p] = angular_dynamics_euler(angles, omega)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%% Angles definition

phi = angles(1);
theta = angles(2);
psi = angles(3);

W = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);...
     0 cos(phi) sin(phi);...
     0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
 
angles_p = W*omega;
end

