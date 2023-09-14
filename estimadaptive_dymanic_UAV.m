function [Test,x] = estimadaptive_dymanic_UAV(x, vcp, vc, v, qd, q, A, B, L, ts)
%                                         vcp, vc, v, chies, K1, K2, L, ts                                   
%  Summary of this function goes here
%  Detailed explanation goes here

a = L(1);
b = L(2);
%% Gain Matrices
A = A*eye(size(v,1));
%% Control error veclocity
ve = tanh(vc -v);
qe = tanh(qd - q);
sigma = ve + A*qe;

vr = sigma + v;
vrp = vcp + A*ve;

mu_l = vr(1);
mu_m = vr(2);
mu_n = vr(3);
w = vr(4);

s1=vrp(1);
s2=vrp(2);
s3=vrp(3);
s4=vrp(4);
%% REFRENCE VELOCITIES


Yu = [s1, b*s4,  0,    0,  0,    0,    0,              0,  0, mu_l, mu_m*w, a*w^2,          0,    0,         0,    0,            0,            0,     0;...
 0,    0, s2, a*s4,  0,    0,    0,              0,  0,    0,          0,         0, mu_l*w, mu_m, b*w^2,    0,            0,            0,     0;...
 0,    0,  0,    0, s3,    0,    0,              0,  0,    0,          0,         0,          0,    0,         0, mu_n,            0,            0,     0;...
 0,    0,  0,    0,  0, b*s1, a*s2, s4*(a^2 + b^2), s4,    0,          0,         0,          0,    0,         0,    0, a*mu_l*w, b*mu_m*w, w];
 
 
 
 
     %% AAPTATIVE CONTROLLER

K = B*eye(19); %fijo 0.1

xp = K*Yu'*sigma;
x = x + xp*ts;

%Test = 1*tanh(Yu*x);
Test = Yu*x;
Test = min(max(Test, -2.5), 2.5);

end
