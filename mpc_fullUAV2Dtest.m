 function [f,solver,args] = mpc_fullUAV2D(bounded, N, L, ts, Q, R)

import casadi.*;

%% Definicion de las restricciones en las acciones de control
f_max = bounded(1); 
f_min = bounded(2);

ux_max = bounded(3); 
ux_min = bounded(4);

%% Generacion de las variables simbolicas de los estados del sistema
y = SX.sym('y'); 
z = SX.sym('z');
phi = SX.sym('phi');
y_p = SX.sym('y_p'); 
z_p = SX.sym('z_p');
phi_p = SX.sym('phi_p');
%% Definicion de cuantos estados en el sistema
states = [y;z;phi;y_p;z_p;phi_p];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
f = SX.sym('ul');
ux = SX.sym('um');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [f;ux]; 
n_control = length(controls);

%% Definicion de los las constantes dl sistema
g = L(1);
m = L(2);
Ixx = L(3);
%% Defincion del sistema pero usando espacios de estados todo el sistema de ecuaciones
aux = zeros(3,2);
B = [aux;
    -(1/m)*sin(states(3)) 0;
    (1/m)*cos(states(3)) 0;
    0 1/Ixx];

A = [zeros(3,3), eye(3);zeros(3,6)]
G = [0;0;0;0;-g;0]

%x_p = A*states+G + B*controls;
R = [states(4) states(5) states(6) 0 -g 0]';
x_p = R + B*controls;

%rhs=(J*controls);

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
    n=2;
    hd = P(n_states*k+1:n_states*k+n); 
    he = X(1:n,k)-hd;
    fcost = he'*Q*he + con'*R*con;
    obj = obj+ fcost;
    
    %obj = obj+(st-P(7*k+1:7*k+4))'*Q*(st-P(7*k+1:7*k+4)) + con'*R*con;
    %% Actualizacion del sistema usando Euler runge kutta
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

args.lbg(1:n_states*(N+1)) = 0;  %-1e-20  %Equality constraints
args.ubg(1:n_states*(N+1)) = 0;  %1e-20   %Equality constraints

%% Inewquality contrains

args.lbx(1:n_states:n_states*(N+1),1) = -inf; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = inf;  %state x upper bound

args.lbx(2:n_states:n_states*(N+1),1) = -inf; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = inf;  %state y upper bound

args.lbx(3:n_states:n_states*(N+1),1) = -inf; %state z lower bound
args.ubx(3:n_states:n_states*(N+1),1) = inf;  %state z upper bound

args.lbx(4:n_states:n_states*(N+1),1) = -inf; %state theta lower bound
args.ubx(4:n_states:n_states*(N+1),1) = inf;  %state theta upper bound

args.lbx(5:n_states:n_states*(N+1),1) = -inf; %state x lower bound
args.ubx(5:n_states:n_states*(N+1),1) = inf;  %state x upper bound

args.lbx(6:n_states:n_states*(N+1),1) = -inf; %state y lower bound
args.ubx(6:n_states:n_states*(N+1),1) = inf;  %state y upper bound


%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(n_states*(N+1)+1:2:n_states*(N+1)+2*N,1) = f_min;  %
args.ubx(n_states*(N+1)+1:2:n_states*(N+1)+2*N,1) = f_max;  %

args.lbx(n_states*(N+1)+2:2:n_states*(N+1)+2*N,1) = ux_min;  %
args.ubx(n_states*(N+1)+2:2:n_states*(N+1)+2*N,1) = ux_max;  % 



end