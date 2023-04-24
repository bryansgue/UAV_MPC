 function [f,solver,args] = mpc_droneKin(bounded, N, L, ts, Q, R)

import casadi.*;

%% Definicion de las restricciones en las acciones de control
ul_max = bounded(1); 
ul_min = bounded(2);

um_max = bounded(3);
um_min = bounded(4);

un_max = bounded(5);
un_min = bounded(6);

w_max = bounded(7); 
w_min = bounded(8);

%% Generacion de las variables simbolicas de los estados del sistema
x = SX.sym('x'); 
y = SX.sym('y');
z = SX.sym('z');
th = SX.sym('th');

%% Definicion de cuantos estados en el sistema
states = [x;y;z;th];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
ul = SX.sym('ul');
um = SX.sym('um');
un = SX.sym('un');
w = SX.sym('w');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [ul;um;un;w]; 
n_control = length(controls);

%% Definicion de los las constantes dl sistema
a = L(1);
b = L(2);

%% Defincion del sistema pero usando espacios de estados todo el sistema de ecuaciones
J = [cos(th) -sin(th) 0 0;
    sin(th) cos(th) 0 0;
    0 0 1 0;
    0 0 0 1];
h_p = J*controls;


%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{h_p}); 
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states));
%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
obj = 0; % Funcion obejtivo la que vamos minimizar
g = [];  % restricciones de estados del problema  de optimizacion

st  = X(:,1); % initial state
g = [g;st-P(1:4)]; % initial condition constraints

%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    st = X(:,k);  con = U(:,k);

    %% Funcion costo a minimizar z
    hd = P(4*k+1:4*k+4);
    he = st-hd;
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
OPT_variables = [reshape(X,4*(N+1),1);reshape(U,4*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;    
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:4*(N+1)) = 0;  %-1e-20  %Equality constraints
args.ubg(1:4*(N+1)) = 0;  %1e-20   %Equality constraints

%% Inewquality contrains


args.lbx(1:4:4*(N+1),1) = -inf; %state x lower bound
args.ubx(1:4:4*(N+1),1) = inf;  %state x upper bound

args.lbx(2:4:4*(N+1),1) = -inf; %state y lower bound
args.ubx(2:4:4*(N+1),1) = inf;  %state y upper bound

args.lbx(3:4:4*(N+1),1) = -inf; %state z lower bound
args.ubx(3:4:4*(N+1),1) = inf;  %state z upper bound

args.lbx(4:4:4*(N+1),1) = -inf; %state theta lower bound
args.ubx(4:4:4*(N+1),1) = inf;  %state theta upper bound


%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(4*(N+1)+1:4:4*(N+1)+4*N,1) = ul_min;  %
args.ubx(4*(N+1)+1:4:4*(N+1)+4*N,1) = ul_max;  %

args.lbx(4*(N+1)+2:4:4*(N+1)+4*N,1) = un_min;  %
args.ubx(4*(N+1)+2:4:4*(N+1)+4*N,1) = un_max;  % 

args.lbx(4*(N+1)+3:4:4*(N+1)+4*N,1) = um_min;  %
args.ubx(4*(N+1)+3:4:4*(N+1)+4*N,1) = um_max;  %

args.lbx(4*(N+1)+4:4:4*(N+1)+4*N,1) = w_min;  %
args.ubx(4*(N+1)+4:4:4*(N+1)+4*N,1) = w_max;  %

end