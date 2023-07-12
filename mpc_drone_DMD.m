 function [f,solver,args] = mpc_drone_DMD(A,B,bounded, N, L, ts, Q, R, obs)

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
psi = SX.sym('psi');
ul = SX.sym('ul');
um = SX.sym('um');
un = SX.sym('un');
w = SX.sym('w');

%% Definicion de cuantos estados en el sistema
states = [x;y;z;psi;ul;um;un;w];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
ul_ref = SX.sym('ul_ref');
um_ref = SX.sym('um_ref');
un_ref = SX.sym('un_ref');
w_ref = SX.sym('w_ref');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [ul_ref;um_ref;un_ref;w_ref]; 
n_control = length(controls);

%% Definicion de los las constantes dl sistema
a = L(1);
b = L(2);

%% Defincion del sistema pero usando espacios de estados todo el sistema de ecuaciones
J = [cos(psi), -sin(psi), 0, -(a*sin(psi)+b*cos(psi));...
     sin(psi), cos(psi), 0,   (a*cos(psi)-b*sin(psi));...
     0, 0, 1, 0;...
     0, 0, 0, 1]; 


%% Definicion del Sistemaa

M = [zeros(4,4),J;...
     zeros(4,4),A];

C = [zeros(4,4);
     B];
    
%% Definicion de kas funciones del sistema
X = SX.sym('X',n_states,(N+1));
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states));
%% vector que incluye el vector de estados y la referencia
x_p=(M*states+C*controls);
f = Function('f',{states,controls},{x_p}); 
%% Vector que representa el problema de optimizacion
obj = 0; % Funcion obejtivo la que vamos minimizar
g = [];  % restricciones de estados del problema  de optimizacion

st  = X(:,1); % initial state
g = [g;st-P(1:n_states)]; % initial condition constraints


he = [];
u = [];

%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    st = X(:,k);  con = U(:,k);

    %% Funcion costo a minimizar z
    hd = P(n_states*k+1:n_states*k+4);       
    he = X(1:4,k)-hd;
    fcost = he'*Q*he + con'*R*con;
    obj = obj+ fcost;
    
%     he = [he;X(1:4,k)-hd];
%     u = [u;con];
    
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

% % Cost final 
% Q = 1*eye(size(he,1));
% R = 0.01*eye(size(u,1));
% 
% % FINAL COST
% obj = he'*Q*he+u'*R*u;

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

% args.lbg(n_states*(N+1)+1:n_states*(N+1)+ 8*(N-1)) = -inf;  
% args.ubg(n_states*(N+1)+1:n_states*(N+1)+ 8*(N-1)) = 0;  

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

args.lbx(7:n_states:n_states*(N+1),1) = -inf; %state z lower bound
args.ubx(7:n_states:n_states*(N+1),1) = inf;  %state z upper bound

args.lbx(8:n_states:n_states*(N+1),1) = -inf; %state theta lower bound
args.ubx(8:n_states:n_states*(N+1),1) = inf;  %state theta upper bound

% args.lbx(9:n_states:n_states*(N+1),1) = 0; %state theta lower bound
% args.ubx(9:n_states:n_states*(N+1),1) = 1;  %state theta upper bound
% 
% args.lbx(10:n_states:n_states*(N+1),1) = 0; %state theta lower bound
% args.ubx(10:n_states:n_states*(N+1),1) = 1;  %state theta upper bound
% 
% args.lbx(11:n_states:n_states*(N+1),1) = 0; %state theta lower bound
% args.ubx(11:n_states:n_states*(N+1),1) = 1;  %state theta upper bound

%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(n_states*(N+1)+1:4:n_states*(N+1)+4*N,1) = ul_min;  %
args.ubx(n_states*(N+1)+1:4:n_states*(N+1)+4*N,1) = ul_max;  %

args.lbx(n_states*(N+1)+2:4:n_states*(N+1)+4*N,1) = um_min;  %
args.ubx(n_states*(N+1)+2:4:n_states*(N+1)+4*N,1) = um_max;  % 

args.lbx(n_states*(N+1)+3:4:n_states*(N+1)+4*N,1) = un_min;  %
args.ubx(n_states*(N+1)+3:4:n_states*(N+1)+4*N,1) = un_max;  %

args.lbx(n_states*(N+1)+4:4:n_states*(N+1)+4*N,1) = w_min;  %
args.ubx(n_states*(N+1)+4:4:n_states*(N+1)+4*N,1) = w_max;  %

 end

 