function [vref,x] = adaptive_OPTI(vcp, vc, v, x,chi, k3, k4, L, ts)
%                                         vcp, vc, v, chies, K1, K2, L, ts                                   
%  Summary of this function goes here
%  Detailed explanation goes here
mu_l = v(1);
mu_m = v(2);
mu_n = v(3);
w = v(4);
a = L(1);
b = L(2);
%% Gain Matrices
K3 = k3*eye(size(v,1));
K4 = k4*eye(size(v,1));
%% Control error veclocity
ve = vc-v;
control = vcp + K3*tanh(inv(K3)*K4*ve);
s1=control(1);
s2=control(2);
s3=control(3);
s4=control(4);
%% REFRENCE VELOCITIES
% vc_1 = vc(1);
% vc_2 = vc(2);
% vc_3 = vc(3);
% vc_4 = vc(4);

mu_l = v(1);
mu_m = v(2);
mu_n = v(3);
w = v(4);
% %

% Y = [s1, s4,     0,       0,     0,       0,       0,                   0,       0,   mu_l, mu_m*w, a*w*w,               0,     0,               0,     0,             0,             0,       0;
%          0,       0, s2, s4,     0,       0,       0,                   0,       0,      0,           0,               0,  mu_l*w,  mu_m,    b*w*w,     0,             0,             0,       0;
%          0,       0,     0,       0, s3,       0,       0,                   0,       0,      0,           0,               0,           0,     0,               0,  mu_n,             0,             0,       0;
%          0,       0,     0,       0,     0, b*s1, a*s2, s4*(a^2 + b^2), s4,    0,           0,               0,           0,     0,               0,     0,  a*mu_l*w,  b*mu_m*w, w];

Y=  [ s1, a*w*s4,  0,          0,  0,          0,          0,  0, mu_l, a*w^2,    0,         0,    0,              0,              0,           0,           0,     0;
      0,          0, s2, b*w*s4,  0,          0,          0,  0,    0,         0, mu_m, b*w^2,    0,              0,              0,           0,           0,     0;
      0,          0,  0,          0, s3,          0,          0,  0,    0,         0,    0,         0, mu_n,              0,              0,           0,           0,     0;
      0,          0,  0,          0,  0, a*w*s1, b*w*s2, s4,    0,         0,    0,         0,    0, b*mu_l*w^2, a*mu_m*w^2, a^2*w^3, b^2*w^3, w];    
     
%% AAPTATIVE CONTROLLER
% [x] = adaptacion(x, Y, ve, ts);
% 
% vref = Y*x;

% 
K = 1*eye(18);
xp = inv(K)*Y'*ve;
x = x + xp*ts;
vref = Y*x;

% vref = Y*chi';



end
