function [omega_p] = angular_dynamics(omega, T, I)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
S_omega = skew_symmetric_matrix(omega);
omega_p = -inv(I)*(-T+S_omega*I*omega);
end

