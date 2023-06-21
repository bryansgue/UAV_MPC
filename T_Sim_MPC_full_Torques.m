% Programa de control predictivo para un drone basado en optimizacion usando Casadi

% Eliminar variables del sistema
clear all;
close all;
clc;

%Generacion de los tiempos del sistema
f = 10 % Hz
ts = 1/f;
tfinal = 30;
t = [0:ts:tfinal];

% Definicion del horizonte de prediccion
N = 10;

% Definicion de las constantes del sistema
g = 9.81;
m = 1.2;
Ixx = 0.0232;
Iyy = 0.0232;
Izz = 0.0468;
X =[m;Ixx;Iyy;Izz];

Tx_max = 0.5;
Ty_max = 0.5;
Tz_max = 0.5;

Tx_min = -Tx_max;
Ty_min = -Ty_max;
Tz_min = -Tz_max;


% Definicion de los limites de las acciondes de control
bounded = [3*m*g; 0; Tx_max; Tx_min; Ty_max; Ty_min; Tz_max; Tz_min];

% Seccion para cargar los parametros dinamicos del sistema

load("chi_values.mat");

val0 = 1.5; % Errores
val1 = 0.8; % Fuerza
val2 = 2; % Torques

% Deficion de la matriz de la matriz de control
Q = eye(3);
Q(1,1) = 1; % X
Q(2,2) = 1; % Y
Q(3,3) = 1; % Z

% Definicion de la matriz de las acciones de control
S = eye(4);
S(1,1) = 1/(3*m*g);
S(2,2) = 1/Tx_max;
S(3,3) = 1/Ty_max;
S(4,4) = 1/Tz_max;

%% a) ESTADOS DEL MPC

x(1)    =  0;
y(1)    =  0;
z(1)    =  0;
phi(1)  =  0*pi/180;
theta(1)=  0;
psi(1)  =  0;

x_p(1)    =  0;
y_p(1)    =  0;
z_p(1)    =  0;
phi_p(1)  =  0;
theta_p(1)=  0;
psi_p(1)  =  0;

x1(:,1) = [x(1);y(1);z(1);phi(1);theta(1);psi(1)];

x2(:,1) = [x_p(1);y_p(1);z_p(1);phi_p(1);theta_p(1);psi_p(1)];

%% ESTADOS DE POSICION PARA SISTEMA
h = [x(1);y(1);z(1);x_p(1);y_p(1);z_p(1)];

angles_b = [phi(1);theta(1);psi(1)];

%% Rotational Matrix
R_NC = Rot_zyx(angles_b);
R_total = zeros(3,3,length(t)+1);
R_total(:,:,1) = R_NC;

omega = [0; 0; 0];

%% Inertial Value
I = [Ixx, 0, 0;...
    0, Iyy, 0;...
    0 0 Izz];
%% GENERAL VECTOR DEFINITION
H = [x1;x2];

% Definicion del vectro de control inicial del sistema
v_N = zeros(N,4);

x_N = repmat(H,1,N+1)';


%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
mul = 10;
[hxd, hyd, hzd, psid, hxdp, hydp, hzdp, psidp] = Trayectorias(3,t,mul);
%% GENERALIZED DESIRED SIGNALS
%psid = 0*psid;

% hxd = -5*ones(1,length(t));
% hyd = 5*ones(1,length(t));
% hzd = 5.5*ones(1,length(t));
hd = [hxd; hyd; hzd; 0*hxd; 0*hyd; 0*hzd; 0*hxd; 0*hyd; 0*hzd; 0*hxd; 0*hyd; 0*hzd];

%% Definicion del optimizador
[f, solver, args] = mpc_fullUAV3D(bounded, N, X, ts, Q, S, val0, val1, val2);

%% Control values torques
F=1.0*m*g;
T = [0.001*cos(t);...
    0.001*sin(t);...
    0.000*ones(1,length(t))];

s = [x(1);y(1);z(1);phi(1);theta(1);psi(1);x_p(1);y_p(1);z_p(1);phi_p(1);theta_p(1);psi_p(1)];

tic

for k=1:length(t)-N
    %% Generacion del; vector de error del sistema
    tic
    he(:,k) = hd(1:3,k) - s(1:3,k);
    
    %tic
    [u_opt,x_opt] = SolverUAV3D_MPC(s(:, k),hd,N,x_N,v_N,args,solver,k);
    %sample(k)=toc;
    
    uc(:,k)= u_opt(1,:)';
    h_N(:,1:3,k) = x_opt(:,1:3);
    
    %% System evolution
    
    s(:,k+1) = UAV_dynamic_Casadi(f,ts, s(:,k),uc(:,k));
    
    %    Actualizacion de los resultados del optimizador para tener una soluciona aproximada a la optima
    v_N = [u_opt(2:end,:);u_opt(end,:)];
    x_N = [x_opt(2:end,:);x_opt(end,:)];
    
    while(toc<ts)
    end
    
    dt(k) = toc;
end
disp("fin");
toc
%%
close all; paso=1;
%a) Parámetros del cuadro de animación
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
x1 = light;
x1.Color=[0.65,0.65,0.65];
x1.Style = 'infinite';
%b) Dimenciones del Robot
Drone_Parameters(0.02);
%c) Dibujo del Robot
%G2=Drone_Plot_3D(0,h(1),h(2),0,0,h(2));hold on

G21=Drone_Plotv2_3D(s(1,1),s(2,1),s(3,1),s(4,1),s(5,1),s(6,1));hold on

G3 = plot3(0,h(1),h(1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on
%   G4 = plot3(0,hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);
G5 = plot3(0,h(1),h(1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on
%plot3(hxd(ubicacion),hyd(ubicacion),hzd(ubicacion),'*r','linewidth',1.5);
view(-45,15);

paso = 1;
for k = 1:paso:length(t)-N
    drawnow
    % delete(G2);
    delete(G3);
    delete(G21);
    delete(G5);
    %   G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),angles_b(1, k), angles_b(2, k), angles_b(3, k));hold on
    G21=Drone_Plotv2_3D(s(1,k),s(2,k),s(3,k),s(4,k),s(5,k),s(6,k));hold on
    
    G3 = plot3(hxd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    
    G5 = plot3(h_N(1:N,1,k),h_N(1:N,2,k),h_N(1:N,3,k),'Color',[100,100,100]/255,'linewidth',0.1);
    ylabel('Y [m]'); xlabel('X [m]'); zlabel('Z [m]');
    
    pause(0)
end

%%
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

plot(t(1:length(he)),he(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(3,:),'Color',[1,88,189]/255,'linewidth',1); hold on;
grid on;
legend({'$\tilde{h_{x}}$','$\tilde{h_{y}}$','$\tilde{h_{z}}$','$\tilde{h_{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);


% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aÃ©reo
figure(3)

subplot(3,1,1)
plot(hxd)
hold on
plot(s(1,:))
legend("yd","hy")
ylabel('y [m]'); xlabel('s [ms]');


subplot(3,1,2)
plot(hyd)
hold on
plot(s(2,:))
legend("yd","hy")
ylabel('y [m]'); xlabel('s [ms]');


subplot(3,1,3)
plot(hzd)
hold on
plot(s(3,:))
grid on
legend("zd","hz")
ylabel('z [m]'); xlabel('s [ms]');



% %%%%%%%%%%%%%
figure(4)

subplot(4,1,1)
plot(uc(1,:))
hold on

legend("F")
ylabel('N'); xlabel('s [ms]');
title('$\textrm{ACCIONES DE CONTROL}$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(uc(2,:))
hold on

legend("ux")
ylabel('Nm'); xlabel('s [ms]');


subplot(4,1,3)
plot(uc(3,:))
hold on

legend("uy")
ylabel('Nm'); xlabel('s [ms]');

subplot(4,1,4)
plot(uc(4,:))
hold on

legend("uz")
ylabel('Nm'); xlabel('s [ms]');

figure(7)

subplot(3,1,1)
plot(s(4,:))
legend("phi")
ylabel('y [m]'); xlabel('s [ms]');


subplot(3,1,2)
plot(s(5,:))
legend("theta")
ylabel('y [m]'); xlabel('s [ms]');


subplot(3,1,3)
plot(s(6,:))
grid on
legend("psi")
ylabel('z [m]'); xlabel('s [ms]');

