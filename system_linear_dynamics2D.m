function [h] = system_linear_dynamics2D(h, F, L, ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = linear_dynamics2D(h, F, L);
k2 = linear_dynamics2D(h + ts/2*k1, F, L); % new
k3 = linear_dynamics2D(h + ts/2*k2, F, L); % new
k4 = linear_dynamics2D(h + ts*k3,  F, L); % new

h = h +ts/6*(k1 +2*k2 +2*k3 +k4); % new
end
