 function [f,solver,args] = mpc_drone_quat(chi_uav,bounded, N, L, ts, Q, R, obs)

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
qw = SX.sym('qw'); 
qx = SX.sym('qx');
qy = SX.sym('qy');
qz = SX.sym('qz');
ul = SX.sym('ul');
um = SX.sym('um');
un = SX.sym('un');
w = SX.sym('w');

%% Definicion de cuantos estados en el sistema
states = [x;y;z;qw;qx;qy;qz;ul;um;un;w];
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
quat = [qw;qx;qy;qz];
J =  QuatToRot_cas(quat);

% INERCIAL MATRIX
M11=chi_uav(1);
M12=0;
M13=0;
M14=b*chi_uav(2);
M21=0;
M22=chi_uav(3) ;
M23=0;
M24=a*chi_uav(4);
M31=0;
M32=0;
M33=chi_uav(5);
M34=0;
M41=b*chi_uav(6);
M42=a*chi_uav(7);
M43=0;
M44=chi_uav(8)*(a^2 + b^2) + chi_uav(9);

M=[M11,M12,M13,M14;...
    M21,M22,M23,M24;...
    M31,M32,M33,M34;...
    M41,M42,M43,M44];

%% CENTRIOLIS MATRIX
C11=chi_uav(10);
C12=w*chi_uav(11);
C13=0;
C14=a*w*chi_uav(12);
C21=w*chi_uav(13);
C22=chi_uav(14);
C23=0;
C24=b*w*chi_uav(15);
C31=0;
C32=0;
C33=chi_uav(16);
C34=0;
C41=a*w*chi_uav(17);
C42=b*w*chi_uav(18);
C43=0;
C44=chi_uav(19);

C=[C11,C12,C13,C14;...
    C21,C22,C23,C24;...
    C31,C32,C33,C34;...
    C41,C42,C43,C44];

%% GRAVITATIONAL MATRIX
G11=0;
G21=0;
G31=0;
G41=0;

G=[G11;G21;G31;G41];
%% Definicion del Sistemaa

%% Evolucion quat
p=0;
q=0;
r=w;

S = [0, -p, -q, -r;...
     p, 0, r, -q;...
     q, -r, 0, p;...
     r, q, -p, 0];
 

%% Definicion del Sistemaa
A = [zeros(3,7),J,zeros(3,1);...
     zeros(4,3), (1/2)* S, zeros(4,4);...
     zeros(4,7),-inv(M)*C];

B = [zeros(7,4);
     inv(M)];
 
%% Definicion de kas funciones del sistema
X = SX.sym('X',n_states,(N+1));
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states));
%% vector que incluye el vector de estados y la referencia
rhs=(A*states+B*controls);
f = Function('f',{states,controls},{rhs}); 
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
     
%     quat_d =  [0.5    ,    0 ,       0   ,0.5];
    quat_d = P(n_states*k+4:n_states*k+7);
    
    q_real = X(4:7,k);
    
    norm_q = norm(q_real);
    q_inv = [q_real(1); -q_real(2:4)] / norm_q;
    
    q_error = quaternionMultiply(q_inv, quat_d);
    
    norm_qe = norm(q_error);
    
    hd = P(n_states*k+1:n_states*k+3);
    he = X(1:3,k)-hd;
    
    %fcost = he'*Q*he + con'*R*con;
     fcost = he'*Q*he + con'*R*con + 1*(1-q_error(1))+ 1*(q_error(2:4)'*q_error(2:4));
%      fcost = he' * Q * he + con' * R * con + (1-norm_qe)^2;
    
    obj = obj+ fcost;
    

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

args.lbx(9:n_states:n_states*(N+1),1) = -inf; %state theta lower bound
args.ubx(9:n_states:n_states*(N+1),1) = inf;  %state theta upper bound

args.lbx(10:n_states:n_states*(N+1),1) = -inf; %state theta lower bound
args.ubx(10:n_states:n_states*(N+1),1) = inf;  %state theta upper bound



args.lbx(11:n_states:n_states*(N+1),1) = -inf; %state theta lower bound
args.ubx(11:n_states:n_states*(N+1),1) = inf;  %state theta upper bound

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

 