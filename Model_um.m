function [vref] = Model_um(u_p, u, chi_uav, L)

% INERCIAL MATRIX
M = function_M(chi_uav,L);
C = function_C(chi_uav,u, L);


vref = M*u_p+C*u;


end