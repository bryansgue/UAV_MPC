 function [f,solver,args] = mpc_fullUAV3D(bounded, N, L, ts, Q, S, val0, val1, val2 )

import casadi.*;

%% Definicion de las restricciones en las acciones de control
f_max = bounded(1); 
f_min = bounded(2);

ux_max = bounded(3); 
ux_min = bounded(4);

uy_max = bounded(5); 
uy_min = bounded(6);

uz_max = bounded(7); 
uz_min = bounded(8);

%% Generacion de las variables simbolicas de los estados del sistema
x = SX.sym('x'); 
y = SX.sym('y');
z = SX.sym('z');
phi = SX.sym('phi'); 
theta = SX.sym('theta');
psi = SX.sym('psi');
x_p = SX.sym('x_p'); 
y_p = SX.sym('y_p');
z_p = SX.sym('z_p');
phi_p = SX.sym('phi_p'); 
theta_p = SX.sym('theta_p');
psi_p = SX.sym('psi_p');
%% Definicion de cuantos estados en el sistema
states = [x;y;z;phi;theta;psi;x_p;y_p;z_p;phi_p;theta_p;psi_p];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
F = SX.sym('F');
ux = SX.sym('ux');
uy = SX.sym('uy');
uz = SX.sym('uz');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [F;ux;uy;uz]; 
n_control = length(controls);

%% Definicion de los las constantes dl sistema
gravity = L(1);
m = L(2);
Ixx = L(3);
Iyy = L(4);
Izz = L(5);

I=eye(3);

RotX = [1 0 0;...
        0 cos(phi) -sin(phi);...
        0 sin(phi) cos(phi)];
    
RotY = [cos(theta) 0 sin(theta);...
        0 1 0;...
        -sin(theta) 0 cos(theta)];
    
RotZ = [cos(psi) -sin(psi) 0;...
        sin(psi) cos(psi) 0;...
        0 0 1];

R = RotZ*RotY*RotX;

%% PARA R
M11 = Ixx ;
M12 = 0 ;
M13 = -Ixx*sin(states(5));
M21 =  0;
M22 =  Izz + Iyy*cos(states(4))^2 - Izz*cos(states(4))^2;
M23 =  cos(states(4))*cos(states(5))*sin(states(4))*(Iyy - Izz);
M31 =  -Ixx*sin(states(5));
M32 =  cos(states(4))*cos(states(5))*sin(states(4))*(Iyy - Izz);
M33 =  Ixx - Ixx*cos(states(5))^2 + Iyy*cos(states(5))^2 - Iyy*cos(states(4))^2*cos(states(5))^2 + Izz*cos(states(4))^2*cos(states(5))^2;


M = [M11 M12 M13;
    M21 M22 M23;
    M31 M32 M33];

C11 = 0;
C12 = (Iyy*theta_p*sin(2*phi))/2 - (Izz*theta_p*sin(2*phi))/2 - Ixx*psi_p*cos(theta) + Iyy*psi_p*cos(theta) - Izz*psi_p*cos(theta) - 2*Iyy*psi_p*cos(phi)^2*cos(theta) + 2*Izz*psi_p*cos(phi)^2*cos(theta);
C13 = Izz*psi_p*cos(phi)*cos(theta)^2*sin(phi) - Iyy*psi_p*cos(phi)*cos(theta)^2*sin(phi);
C21 =  Izz*theta_p*sin(2*phi) - Iyy*theta_p*sin(2*phi) + Ixx*psi_p*cos(theta) - Iyy*psi_p*cos(theta) + Izz*psi_p*cos(theta) + 2*Iyy*psi_p*cos(phi)^2*cos(theta) - 2*Izz*psi_p*cos(phi)^2*cos(theta);
C22 = 0;
C23 =  (Iyy*psi_p*sin(2*theta))/2 - (Ixx*psi_p*sin(2*theta))/2 - Iyy*psi_p*cos(phi)^2*cos(theta)*sin(theta) + Izz*psi_p*cos(phi)^2*cos(theta)*sin(theta);
C31 =  Izz*theta_p*cos(theta) - Iyy*theta_p*cos(theta) - Ixx*theta_p*cos(theta) + 2*Iyy*theta_p*cos(phi)^2*cos(theta) - 2*Izz*theta_p*cos(phi)^2*cos(theta) + 2*Iyy*psi_p*cos(phi)*cos(theta)^2*sin(phi) - 2*Izz*psi_p*cos(phi)*cos(theta)^2*sin(phi);
C32 =  Ixx*psi_p*sin(2*theta) - Iyy*psi_p*sin(2*theta) + 2*Iyy*psi_p*cos(phi)^2*cos(theta)*sin(theta) - 2*Izz*psi_p*cos(phi)^2*cos(theta)*sin(theta) - Iyy*theta_p*cos(phi)*sin(phi)*sin(theta) + Izz*theta_p*cos(phi)*sin(phi)*sin(theta);
C33 =  0;

C = [C11 C12 C13;
     C21 C22 C23;
     C31 C32 C33];

M_bar = [m*I zeros(3,3);zeros(3,3) M];
C_bar = [zeros(3,3) zeros(3,3);zeros(3,3) C];
G_bar = [0;0;gravity*m;0;0;0];

estados = [states(7);states(8);states(9);states(10);states(11);states(12)];
    
aux1 = R*[0;0;controls(1)];
aux2 = [controls(2);controls(3);controls(4)];

aux = inv(M_bar)*([aux1;aux2]-C_bar*estados-G_bar);

h_p = [states(7);
       states(8);
       states(9);
       states(10);
       states(11);
       states(12);
       aux];

%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{h_p}); 

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
  
    fcost = val0*(he'*Q*he) + val1*(con(1)'* S(1,1) *con(1)) + val2*(con(2:4)'*S(2:4,2:4)*con(2:4)) ;
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
args.lbx(n_states*(N+1)+1:4:n_states*(N+1)+4*N,1) = f_min;  %
args.ubx(n_states*(N+1)+1:4:n_states*(N+1)+4*N,1) = f_max;  %

args.lbx(n_states*(N+1)+2:4:n_states*(N+1)+4*N,1) = ux_min;  %
args.ubx(n_states*(N+1)+2:4:n_states*(N+1)+4*N,1) = ux_max;  % 

args.lbx(n_states*(N+1)+3:4:n_states*(N+1)+4*N,1) = uy_min;  %
args.ubx(n_states*(N+1)+3:4:n_states*(N+1)+4*N,1) = uy_max;  % 

args.lbx(n_states*(N+1)+4:4:n_states*(N+1)+4*N,1) = uz_min;  %
args.ubx(n_states*(N+1)+4:4:n_states*(N+1)+4*N,1) = uz_max;  % 



end