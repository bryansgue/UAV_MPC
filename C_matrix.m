function [C] = C_matrix(chi,euler,euler_p)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
phi = euler(1);
theta = euler(2);
psi = euler(3);

phi_p = euler_p(1);
theta_p = euler_p(2);
psi_p = euler_p(3);

I_xx = chi(1);
I_yy = chi(2);
I_zz = chi(3);

C = [                                                                                                                                     0,                                                                                                               theta_p*sin(2*phi)*(I_yy/2 - I_zz/2) - (psi_p*cos(theta)*(I_xx + I_yy*cos(2*phi) - I_zz*cos(2*phi)))/2,                                                                 - (theta_p*cos(theta)*(I_xx + I_yy*(2*cos(phi)^2 - 1) - I_zz*(2*cos(phi)^2 - 1)))/2 - psi_p*cos(phi)*sin(phi)*cos(theta)^2*(I_yy - I_zz);...
                                (psi_p*cos(theta)*(I_xx + I_yy*cos(2*phi) - I_zz*cos(2*phi)))/2 - (theta_p*sin(2*phi)*(I_yy - I_zz))/2,                                                                                                                                                                                  -(phi_p*sin(2*phi)*(I_yy - I_zz))/2,                                                          (phi_p*cos(theta)*(I_xx + I_yy*cos(2*phi) - I_zz*cos(2*phi)))/2 - psi_p*cos(theta)*sin(theta)*(I_xx - I_zz - I_yy*sin(phi)^2 + I_zz*sin(phi)^2);...
psi_p*cos(phi)*sin(phi)*cos(theta)^2*(I_yy - I_zz) - (theta_p*cos(theta)*(I_xx - I_yy*(2*cos(phi)^2 - 1) + I_zz*(2*cos(phi)^2 - 1)))/2, psi_p*cos(theta)*sin(theta)*(I_xx - I_zz - I_yy*sin(phi)^2 + I_zz*sin(phi)^2) - (phi_p*cos(theta)*(I_xx - I_yy*(2*cos(phi)^2 - 1) + I_zz*(2*cos(phi)^2 - 1)))/2 - theta_p*cos(phi)*sin(phi)*sin(theta)*(I_yy - I_zz), phi_p*(I_yy*cos(phi)*sin(phi)*cos(theta)^2 - I_zz*cos(phi)*sin(phi)*cos(theta)^2) - theta_p*(I_zz*cos(phi)^2*cos(theta)*sin(theta) - I_xx*cos(theta)*sin(theta) + I_yy*sin(phi)^2*cos(theta)*sin(theta))];
 
end

