function [h] = system_linear_dynamics(h, R, F, L, ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = linear_dynamics(h, R, F, L);
k2 = linear_dynamics(h + ts/2*k1, R, F, L); % new
k3 = linear_dynamics(h + ts/2*k2, R, F, L); % new
k4 = linear_dynamics(h + ts*k3, R,  F, L); % new

h = h +ts/6*(k1 +2*k2 +2*k3 +k4); % new
end
