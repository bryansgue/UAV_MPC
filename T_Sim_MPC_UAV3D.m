% Programa de control predictivo para un drone basado en optimizacion usando Casadi

% Eliminar variables del sistema
clear all;
close all;
clc;

%Generacion de los tiempos del sistema
f = 10 % Hz 
ts = 1/f; 
tfinal = 20;
t = [0:ts:tfinal];
 
% Definicion del horizonte de prediccion
N = 10; 

% Definicion de las constantes del sistema
g = 9.8;
m = 1.2;
Ixx = 0.0232;
Iyy = 0.0232;
Izz = 0.0468;
L =[g;m;Ixx;Iyy;Izz];

% Definicion de los limites de las acciondes de control
bounded = [2*m*g; 0; 0.15; -0.15; 0.15; -0.15; 0.15; -0.15];

% Seccion para cargar los parametros dinamicos del sistema

load("chi_values.mat");

% Deficion de la matriz de la matriz de control
Q = eye(3);
Q(1,1) = 1.5; % X
Q(2,2) = 2; % Y
Q(3,3) = 5; % Z

val0 = 100  ; % Errores
val1 = 250; % Fuerza
val2 = 300; % Torques
% Definicion de la matriz de las acciones de control
S = eye(4);

S(1,1) = 1/(2*m*g);
S(2,2) = 1/0.15;
S(3,3) = 1/0.15;
S(4,4) = 1/0.15;

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

% hxd = 3*ones(1,length(t));
% hyd = 3*ones(1,length(t));
% hzd = 6.5*ones(1,length(t));
hd = [hxd; hyd; hzd; 0*hxd; 0*hyd; 0*hzd; 0*hxd; 0*hyd; 0*hzd; 0*hxd; 0*hyd; 0*hzd];


%% Ganancia Compensacion Dinamica
K1 = 1;
K2 = 1;

%% Definicion del optimizador
[f, solver, args] = mpc_fullUAV3D(bounded, N, L, ts, Q, S, val0, val1, val2);

%% Control values torques
F=1.0*m*g;
T = [0.001*cos(t);...
     0.001*sin(t);...
     0.000*ones(1,length(t))];
H_aux = [x(1);y(1);z(1);phi(1);theta(1);psi(1);x_p(1);y_p(1);z_p(1);phi_p(1);theta_p(1);psi_p(1)];
tic
for k=1:length(t)-N   
%% Generacion del; vector de error del sistema
    tic
    he(:,k) = hd(1:3,k) - H_aux(1:3,k);
    
    %tic
    [u_opt,x_opt] = SolverUAV3D_MPC(H_aux(:, k),x2,hd,N,x_N,v_N,args,solver,k);
    %sample(k)=toc;
    
    uc(:,k)= u_opt(1,:)';
    h_N(:,1:3,k) = x_opt(:,1:3);
    
    F = uc(1,k);
    T(:,k) = uc(2:end,k);
    %disp(uc(:,k))
      
    %% System evolution
%     h(:,k+1) = system_linear_dynamics(h(:,k), R_total(:,:,k), F, L, ts);
%     R_total(:,:, k+1) = system_angular_dynamics_r(R_total(:, :, k), omega(:, k), ts);
%     angles_b(:, k+1) = system_angular_dynamics_euler(angles_b(:, k), omega(:,k), ts);
%     omega(:, k+1) = system_angular_dynamics(omega(:, k), T(:, k), I, ts);
%    
   % H_aux(:, k+1) = H_aux(:, k) + full(f(H_aux(:, k),[F;T(:,k)]))*ts;
    
    k1 = full(f(H_aux(:, k),[F;T(:,k)]));
    k2 = full(f(H_aux(:, k) + ts/2*k1,[F;T(:,k)]));
    k3 = full(f(H_aux(:, k) + ts/2*k2,[F;T(:,k)]));
    k4 = full(f(H_aux(:, k) + ts*k3,[F;T(:,k)]));

    H_aux(:, k+1) = H_aux(:, k) +ts/6*(k1 +2*k2 +2*k3 +k4); % new
    
    x1 = H_aux(1:6, k+1);
    x2 = H_aux(7:12, k+1);
    


%     phi = angles_b(1,k+1);
%     theta = angles_b(2,k+1);
%     psi = angles_b(3,k+1);
% 
%     W = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);...
%      0 cos(phi) sin(phi);...
%      0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
%  
%     angles_p = W*omega(:,k+1);
%     
%     x1(:,k+1) = [h(1:3,k+1);angles_b(:,k+1)];
%     x2(:,k+1) = [h(4:6,k+1); angles_p];
% %     
% %     
% %             
% % %    Actualizacion de los resultados del optimizador para tener una soluciona aproximada a la optima
     v_N = [u_opt(2:end,:);u_opt(end,:)];
     x_N = [x_opt(2:end,:);x_opt(end,:)];
     
    
    while(toc<ts)
    end
    1/toc
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
    
    G21=Drone_Plotv2_3D(H_aux(1,1),H_aux(2,1),H_aux(3,1),H_aux(4,1),H_aux(5,1),H_aux(6,1));hold on

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
    G21=Drone_Plotv2_3D(H_aux(1,k),H_aux(2,k),H_aux(3,k),H_aux(4,k),H_aux(5,k),H_aux(6,k));hold on
    
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
plot(H_aux(1,:))
legend("yd","hy")
ylabel('y [m]'); xlabel('s [ms]');


subplot(3,1,2)
plot(hyd)
hold on
plot(H_aux(2,:))
legend("yd","hy")
ylabel('y [m]'); xlabel('s [ms]');


subplot(3,1,3)
plot(hzd)
hold on
plot(H_aux(3,:))
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
plot(H_aux(4,:))
legend("phi")
ylabel('y [m]'); xlabel('s [ms]');


subplot(3,1,2)
plot(H_aux(5,:))
legend("theta")
ylabel('y [m]'); xlabel('s [ms]');


subplot(3,1,3)
plot(H_aux(6,:))
grid on
legend("psi")
ylabel('z [m]'); xlabel('s [ms]');

