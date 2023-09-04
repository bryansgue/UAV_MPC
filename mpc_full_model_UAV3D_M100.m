 function [f,solver,args] = mpc_full_model_UAV3D_M100(bounded, N, chi, ts, K1, K2, val0, val1, val2 )

import casadi.*;

%% Definicion de las restricciones en las acciones de control
zp_max = bounded(1); 
zp_min = bounded(2);

phi_max = bounded(3); 
phi_min = bounded(4);

theta_max = bounded(5); 
theta_min = bounded(6);

psi_max = bounded(7); 
psi_min = bounded(8);

%% Generacion de las variables simbolicas de los estados del sistema
hx = SX.sym('hx'); 
hy = SX.sym('hy');
hz = SX.sym('hz');
phi = SX.sym('phi'); 
theta = SX.sym('theta');
psi = SX.sym('psi');
hx_p = SX.sym('hx_p'); 
hy_p = SX.sym('hy_p');
hz_p = SX.sym('hz_p');
phi_p = SX.sym('phi_p'); 
theta_p = SX.sym('theta_p');
psi_p = SX.sym('psi_p');
%% Definicion de cuantos estados en el sistema

states = [hx;hy;hz;phi;theta;psi;hx_p;hy_p;hz_p;phi_p;theta_p;psi_p];
n_states = length(states);

h = [hx;hy;hz];
euler = [phi;theta;psi];

v = [hx_p;hy_p;hz_p];
euler_p = [phi_p;theta_p;psi_p];

x1 = [hx;hy;hz;phi;theta;psi];
x2 = [hx_p;hy_p;hz_p;phi_p;theta_p;psi_p];

%% Generacion de las variables simbolicas de las acciones del control del sistema
zp_ref = SX.sym('F');
phi_ref = SX.sym('ux');
theta_ref = SX.sym('uy');
psi_p_ref = SX.sym('uz');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [zp_ref;phi_ref;theta_ref;psi_p_ref]; 
n_control = length(controls);

%% PARA R
% g = 9.81;
% m = chi(1);
% 
% R = Rot_zyx(q(4:6));
% 
% Mbar = M_matrix_bar(chi,q);
% Cbar = C_matrix_bar(chi,q,q_p);
% Gbar = G_matrix_bar(chi,q);
% 
% S = S_fuction(chi);
% Q = Q_fuction(chi);
% E = E_fuction(chi);
% T = T_fuction(chi);
% B = [0;0;m*g;0;0;0];
% 
% R_T = [R*T(1:3,1:3) T(1:3,4:6);T(4:6,1:3) T(4:6,4:6)];
% 
% Aux = (S*[0;0;controls]-Q*q-E*q_p+B);
% Aux1 = R*Aux(1:3,1);
% Aux2 = Aux(4:6,1);
% 
% Input_model = [Aux1;Aux2];
% 
% %inv_uax = inv_M_RT(chi,q);
% 
% q_pp = inv(Mbar+R_T)*(Input_model-Cbar*q_p-Gbar);
% %q_pp = inv_uax*(Input_model-Cbar*q_p-Gbar);
% 
% s_p = [q_p;
%        q_pp];
   
%% Model


u_ref = [0;0;zp_ref];
T_ref = [phi_ref;theta_ref;psi_p_ref];

m = chi(25);

I = eye(3);
[A1,B1,Q1,G1,A2,B2,C2,D2,E2,G_tras] = Matrices(chi);

R = Rot_zyx(euler);

phi_p = euler_p(1);
theta_p = euler_p(2);
psi_p = euler_p(3);

c_1 = phi_p*theta_p;
c_2 = phi_p*psi_p;
c_3 = theta_p*psi_p;

sigma = [c_1;c_2;c_3];

M_rot = M_matrix(chi,euler);
C_rot = C_matrix(chi,euler,euler_p);


% Ecuacion de estados

x1_p = x2;

x2_p_a = pinv(m*I+R*Q1)*(R*(A1*u_ref-B1*v+G1)-G_tras);
x2_p_b = pinv(M_rot+E2)*(A2*T_ref + B2*euler + C2*euler_p - C_rot*euler_p + D2*sigma);


x2_p = [x2_p_a;x2_p_b];

x_p = [x1_p; x2_p];


%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{x_p}); 

X = SX.sym('X',n_states,(N+1));
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states));
%% vector que incluye el vector de estados y la referencia


%% Vector que representa el problema de optimizacion
obj = 0; % Funcion obejtivo la que vamos minimizar
g = [];  % restricciones de estados del problema  de optimizacion

st  = X(:,1); % initial state
g = [g;st-P(1:n_states)];

%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    st = X(:,k);  con = U(:,k);

    %% Funcion costo a minimizar z
    n=3;
    hd = P(n_states*k+1:n_states*k+n); 
    he = (X(1:n,k)-hd);

    fcost = val0*(he'*K1*he) + val1*(con(1)'* K2(1,1) *con(1)) + val2*(con(2:4)'*K2(2:4,2:4)*con(2:4)) ;
    obj = obj+ fcost;
    
    %obj = obj+(st-P(7*k+1:7*k+4))'*Q*(st-P(7*k+1:7*k+4)) + con'*R*con;
    %% Actualizacion del sistema usando Euler runge kutta
    st_next1 = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + ts/2*k1, con); % new
    k3 = f(st + ts/2*k2, con); % new
    k4 = f(st + ts*k3, con); % new
    st_next_RK4=st +ts/6*(k1 +2*k2 +2*k3 +k4); % new 
    
    %% Restricciones del sistema se =basan en el modelo del sistema
    g = [g;st_next1-st_next_RK4]; 
end



% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_control*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:n_states*(N+1)) = 1e-20;  %-1e-20  %Equality constraints
args.ubg(1:n_states*(N+1)) = 1e-20;  %1e-20   %Equality constraints

%% Inewquality contrains

args.lbx(1:n_states:n_states*(N+1),1) = -inf; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = inf;  %state x upper bound

args.lbx(2:n_states:n_states*(N+1),1) = -inf; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = inf;  %state y upper bound

args.lbx(3:n_states:n_states*(N+1),1) = -inf; %state z lower bound
args.ubx(3:n_states:n_states*(N+1),1) = inf;  %state z upper bound

args.lbx(4:n_states:n_states*(N+1),1) = -0.611; %state phi lower bound
args.ubx(4:n_states:n_states*(N+1),1) = 0.611;  %state phi upper bound

args.lbx(5:n_states:n_states*(N+1),1) = -0.611; %state theta lower bound
args.ubx(5:n_states:n_states*(N+1),1) = 0.611;  %state theta upper bound

args.lbx(6:n_states:n_states*(N+1),1) = -pi; %state psi lower bound
args.ubx(6:n_states:n_states*(N+1),1) = pi;  %state psi upper bound

args.lbx(7:n_states:n_states*(N+1),1) = -inf; %state xp lower bound
args.ubx(7:n_states:n_states*(N+1),1) = inf;  %state xp upper bound

args.lbx(8:n_states:n_states*(N+1),1) = -inf; %state yp lower bound
args.ubx(8:n_states:n_states*(N+1),1) = inf;  %state yp upper bound

args.lbx(9:n_states:n_states*(N+1),1) = -inf; %state zp lower bound
args.ubx(9:n_states:n_states*(N+1),1) = inf;  %state zp upper bound

args.lbx(10:n_states:n_states*(N+1),1) = -5/6 ; %state phi_p lower bound
args.ubx(10:n_states:n_states*(N+1),1) = 5/6;  %state phi_p upper bound

args.lbx(11:n_states:n_states*(N+1),1) = -5/6; %state theta_p lower bound
args.ubx(11:n_states:n_states*(N+1),1) = 5/6;  %state theta_p upper bound

args.lbx(12:n_states:n_states*(N+1),1) = -5/6; %state psi_p lower bound
args.ubx(12:n_states:n_states*(N+1),1) = 5/6;  %state psi_p upper bound

%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(n_states*(N+1)+1:4:n_states*(N+1)+4*N,1) = zp_min;  %
args.ubx(n_states*(N+1)+1:4:n_states*(N+1)+4*N,1) = zp_max;  %

args.lbx(n_states*(N+1)+2:4:n_states*(N+1)+4*N,1) = phi_min;  %
args.ubx(n_states*(N+1)+2:4:n_states*(N+1)+4*N,1) = phi_max;  % 

args.lbx(n_states*(N+1)+3:4:n_states*(N+1)+4*N,1) = theta_min;  %
args.ubx(n_states*(N+1)+3:4:n_states*(N+1)+4*N,1) = theta_max;  % 

args.lbx(n_states*(N+1)+4:4:n_states*(N+1)+4*N,1) = psi_min;  %
args.ubx(n_states*(N+1)+4:4:n_states*(N+1)+4*N,1) = psi_max;  % 



end