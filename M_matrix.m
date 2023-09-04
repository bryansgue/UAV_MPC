function [M] = M_matrix(chi,euler)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
phi = euler(1);
theta = euler(2);
psi = euler(3);

I_xx = chi(1);
I_yy = chi(2);
I_zz = chi(3);

M = [            I_xx,                                          0,                                                                -I_xx*sin(theta);...
                   0,   I_yy*cos(phi)^2 - I_zz*cos(phi)^2,                                      cos(phi)*sin(phi)*cos(theta)*(I_yy - I_zz);...
    -I_xx*sin(theta), cos(phi)*sin(phi)*cos(theta)*(I_yy - I_zz), I_zz*cos(phi)^2*cos(theta)^2 + I_yy*sin(phi)^2*cos(theta)^2 + I_xx*sin(theta)^2];

end

