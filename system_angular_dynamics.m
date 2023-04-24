function [omega] = system_angular_dynamics(omega, T, I, ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

k1 = angular_dynamics(omega, T, I);
k2 = angular_dynamics(omega + ts/2*k1, T, I);
k3 = angular_dynamics(omega + ts/2*k2, T, I);
k4 = angular_dynamics(omega + ts*k3, T, I); % new

omega = omega +ts/6*(k1 +2*k2 +2*k3 +k4); % new
end