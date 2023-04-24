function [f,solver,args] = optimization_trayectory(bounded, N, a, ts, Q, R)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% CasADi v3.4.
addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');

import casadi.*; 
%% Definicion de las restricciones en las acciones de control
u_max = bounded(1); 
u_min = bounded(2);
w_max = bounded(3); 
w_min = bounded(4);

%% Defincion de los estados del sistema
x = SX.sym('x'); y = SX.sym('y'); th = SX.sym('th');
states = [x;y;th]; n_states = length(states);

%% Defincion de las acciones de control 
u = SX.sym('u'); w = SX.sym('w');
controls = [u;w]; n_controls = length(controls);
rhs = [u*cos(th)-a*sin(th)*w;u*sin(th)+a*cos(th)*w;w]; % system r.h.s

%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{rhs}); 
U = SX.sym('U',n_controls,N); % Decision variables (controls)
%P = SX.sym('P',n_states + N*(n_states));
P = SX.sym('P',n_states + N*(n_states+n_controls));

%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
obj = 0; % Objective function
g = [];  % constraints vector

st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints

%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    st = X(:,k);  con = U(:,k);

    %% Funcion costo a minimizar
    obj = obj+(st-P(5*k-1:5*k+1))'*Q*(st-P(5*k-1:5*k+1))+(con-P(5*k+2:5*k+3))'*R*(con-P(5*k+2:5*k+3)); 
    %obj = obj+(st-P(3*k+1:3*k+3))'*Q*(st-P(3*k+1:3*k+3)) + con'*R*con;
    
    % El numero 3 es por los 3 estados a controladr a los lazrgo del tiempo
    
    %% Actualizacion del sistema usando Euler solamente
    st_next = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + ts/2*k1, con); % new
    k3 = f(st + ts/2*k2, con); % new
    k4 = f(st + ts*k3, con); % new
    st_next_RK4=st +ts/6*(k1 +2*k2 +2*k3 +k4); % new 
    
    %% Restricciones del sistema se =basan en el modelo del sistema
    g = [g;st_next-st_next_RK4]; 
end

% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;

args.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbx(1:3:3*(N+1),1) = -inf; %state x lower bound
args.ubx(1:3:3*(N+1),1) = inf; %state x upper bound

args.lbx(2:3:3*(N+1),1) = -inf; %state y lower bound
args.ubx(2:3:3*(N+1),1) = inf; %state y upper bound

args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = u_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) =  u_max; %v upper bound

args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = w_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = w_max; %omega upper bound
end

