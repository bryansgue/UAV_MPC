% Programa de control predictivo para un drone basado en optimizacion usando Casadi

% Eliminar variables del sistema
clear all;
close all;
clc;

%Generacion de los tiempos del sistema
f = 10;% Hz
ts = 1/f;
tfinal = 15;
t = [0:ts:tfinal];

% Definicion del horizonte de prediccion
N = 5;

% Definicion de las constantes del sistema
g = 9.81;

chi_vel = [ 0.0002    2.2455    2.1997    1.0160];
chi_eul = [-0.1140   -4.6592   -2.2340   62.2132  -62.3365  -14.2061   -7.0005    2.8220    3.1867    2.3961   15.9205  -18.1003   -2.9719   -4.1456 -2.0703   -0.4450    2.9815   31.6103  1.8608  -20.0883   21.0181  -35.1003   -0.3361   13.4542    ];

chi_model = [chi_eul, chi_vel];

zp_ref_max = 3;
phi_max = 0.7;
theta_max = 0.7;
psi_max = 3;

zp_ref_min = -zp_ref_max;
phi_min = -phi_max;
theta_min = -theta_max;
psi_min = -psi_max;


% Definicion de los limites de las acciondes de control
bounded = [zp_ref_max; zp_ref_min; phi_max; phi_min; theta_max; theta_min; psi_max; psi_min];

% Seccion para cargar los parametros dinamicos del sistema

% load("chi_values.mat");

val0 = 3; % Errores
val1 = 1; % Fuerza
val2 = 1; % Torques

% Deficion de la matriz de la matriz de control
K1 = eye(3);
K1(1,1) = 1; % X
K1(2,2) = 1; % Y
K1(3,3) = 1; % Z

% Definicion de la matriz de las acciones de control
K2 = eye(4);
K2(1,1) = 1/zp_ref_max;
K2(2,2) = 1/phi_max;
K2(3,3) = 1/theta_max;
K2(4,4) = 1/psi_max;

%% a) ESTADOS DEL MPC

x(1)    =  0;
y(1)    =  0;
z(1)    =  0;
phi(1)  =  0;
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


%% GENERAL VECTOR DEFINITION
H = [x1;x2];

% Definicion del vectro de control inicial del sistema
v_N = zeros(N,4);

x_N = repmat(H,1,N+1)';


%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
mul = 15;
[hxd, hyd, hzd, psid, hxdp, hydp, hzdp, psidp] = Trayectorias(3,t,mul);
%% GENERALIZED DESIRED SIGNALS

hd = [hxd; hyd; hzd; 0*hxd; 0*hyd; 0*hzd; 0*hxd; 0*hyd; 0*hzd; 0*hxd; 0*hyd; 0*hzd];

%% Definicion del optimizador
[f, solver, args] = mpc_full_model_UAV3D_M100(bounded, N, chi_model, ts, K1, K2, val0, val1, val2);

%% Control values torques

s = [x(1);y(1);z(1);phi(1);theta(1);psi(1);x_p(1);y_p(1);z_p(1);phi_p(1);theta_p(1);psi_p(1)];

tic

% uc = [0*ones(1,length(t));
%     0.05*sin(t);
%     0.05*cos(t);
%     0.01*sin(t)];


for k=1:length(t)-N
    %% Generacion del; vector de error del sistema
    
    he(:,k) = hd(1:3,k) - s(1:3,k);
    %%
    tic
    [u_opt,x_opt] = SolverUAV3D_MPC(s(:, k),hd,N,x_N,v_N,args,solver,k);
    toc;
    
    uc(:,k)= u_opt(1,:)';
    h_N(:,1:3,k) = x_opt(:,1:3);
    
    %% System evolution
    
    s(:,k+1) = UAV_dynamic_Casadi(f,ts, s(:,k),uc(:,k))
    
    %    Actualizacion de los resultados del optimizador para tener una soluciona aproximada a la optima
    v_N = [u_opt(2:end,:);u_opt(end,:)];
    x_N = [x_opt(2:end,:);x_opt(end,:)];
    
    while(toc<ts)
    end
    k
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
G2 = plot3(s(1,1),s(2,1),s(3,1),'Color',[32,185,29]/255,'linewidth',1.5);
G21=Drone_Plotv2_3D(s(1,1),s(2,1),s(3,1),s(4,1),s(5,1),s(6,1));hold on

G3 = plot3(0,h(1),h(1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on
%   G4 = plot3(0,hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);
G5 = plot3(0,h(1),h(1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on
%plot3(hxd(ubicacion),hyd(ubicacion),hzd(ubicacion),'*r','linewidth',1.5);
view(-45,15);

paso = 1;
for k = 1:paso:length(t)-N
    drawnow
    delete(G2);
    delete(G3);
    delete(G21);
    delete(G5);
    
    G21=Drone_Plotv2_3D(s(1,k),s(2,k),s(3,k),s(4,k),s(5,k),s(6,k));hold on
    G2 = plot3(s(1,1:k),s(2,1:k),s(3,1:k),'--','Color',[155,126,125]/255,'linewidth',0.5);
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
legend("xd","hx")
ylabel('x [m]'); 


subplot(3,1,2)
plot(hyd)
hold on
plot(s(2,:))
legend("yd","hy")
ylabel('y [m]'); 

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


figure(9)


plot(s(1,:)); hold on
plot(s(2,:)); hold on
plot(s(3,:)); hold on



