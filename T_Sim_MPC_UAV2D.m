% Programa de control predictivo para un drone basado en optimizacion usando Casadi

% Eliminar variables del sistema
clear all;
close all;
clc;

%Generacion de los tiempos del sistema
f = 30 % Hz 
ts = 1/f; 
tfinal = 10;
t = [0:ts:tfinal];
 
% Definicion del horizonte de prediccion
N = f; 

% Definicion de las constantes del sistema
g = 9.8;
m = 1;
Ixx = 0.02;
L =[g;m;Ixx];

% Definicion de los limites de las acciondes de control
val = 80;
bounded = [2*m*g; 0; val*0.1; -val*0.1];

% Seccion para cargar los parametros dinamicos del sistema

load("chi_values.mat");

% Deficion de la matriz de la matriz de control
Q = 10*eye(3);

% Definicion de la matriz de las acciones de control
R = 0.000001*eye(2);

% Definicion de los estados iniciales del sistema
x = 0;
y = 0;
z = 0;
phi(1,1) = (180*pi/180);

%% a) Posiciones iniciales del UAV

hy(1) = 0; 
hz(1) = 1; 
phi(1)= 0;

x1=[hy(1);hz(1);phi(1)]

x2=[0;0;0];

%% GENERAL VECTOR DEFINITION
h = [x1;x2];

% Definicion del vectro de control inicial del sistema
v_N = zeros(N,2);

H0 = repmat(h,1,N+1)'; 
x_N = H0;

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
mul = 10;
[hxd, hyd, hzd, psid, hxdp, hydp, hzdp, psidp] = Trayectorias(3,t,mul);
%% GENERALIZED DESIRED SIGNALS
psid = -1.5*ones(1,length(t));
Q(3,3) = 0;
hd = [hyd; hzd; psid;hydp;hzdp;psidp];

%% Velocidad inicial real del UAV

chi_estimados(:,1) = chi';
chi_real(:,1) = chi';
%% Ganancia Compensacion Dinamica
K1 = 1;
K2 = 1;

%% Definicion del optimizador
[f, solver, args] = mpc_fullUAV2D(bounded, N, L, ts, Q, R);

tic
for k=1:length(t)-N   
%% Generacion del; vector de error del sistema

    he(:,k)=hd(1:2,k)-h(1:2,k);
    
    tic
    [u_opt,x_opt] = SolverUAV2D_MPC(x1(:,k),x2(:,k),hd,N,x_N,v_N,args,solver,k);
    sample(k)=toc;
    
    uc(:,k)= u_opt(1,:)';
    h_N(:,1:3,k) = x_opt(:,1:3);
    
    %% Dinamica del sistema 
    h(:,k+1) = system_linear_dynamics2D(h(:,k), uc(:, k), L, ts);
    x1(:,k+1) = h(1:3,k+1);
    x2(:,k+1) = h(4:6,k+1);
            
    %% Actualizacion de los resultados del optimizador para tener una soluciona aproximada a la optima
    v_N = [u_opt(2:end,:);u_opt(end,:)];
    x_N = [x_opt(2:end,:);x_opt(end,:)];

end
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
    G2=Drone_Plot_3D(0,h(1),h(2),0,0,h(2));hold on

    G3 = plot3(0,h(1),h(1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   
    G4 = plot3(0,hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);
    G5 = Drone_Plot_3D(0,h(1),h(2),0,0,h(3));hold on
    %plot3(hxd(ubicacion),hyd(ubicacion),hzd(ubicacion),'*r','linewidth',1.5);
view(90,0);

    paso = 1;
for k = 1:paso:length(t)-N
    drawnow
    delete(G2);
    delete(G3);
%    delete(G4);
    delete(G5);
    G2=Drone_Plot_3D(0,h(1,k),h(2,k),h(3, k), 0, 0);hold on
    
    G3 = plot3(0*hyd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
%    G4 = plot3(0,h(1:k),h(2:k),'-.','Color',[56,171,217]/255,'linewidth',1.5);
    G5 = plot3(0*h_N(1:N,1,k),h_N(1:N,1,k),h_N(1:N,2,k),'Color',[100,100,100]/255,'linewidth',0.1);

    
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

grid on;
legend({'$\tilde{h_{x}}$','$\tilde{h_{y}}$','$\tilde{h_{z}}$','$\tilde{h_{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);


% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aÃ©reo
figure(3)

subplot(4,1,1)


subplot(4,1,2)
plot(hyd)
hold on
plot(h(1,:))
legend("yd","hy")
ylabel('y [m]'); xlabel('s [ms]');


subplot(4,1,3)
plot(hzd)
hold on
plot(h(2,:))
grid on
legend("zd","hz")
ylabel('z [m]'); xlabel('s [ms]');

subplot(4,1,4)
plot(psid)
hold on
plot(h(3,:))
grid on
legend("zd","hz")
ylabel('z [m]'); xlabel('s [ms]');


% %%%%%%%%%%%%%
figure(4)

plot(uc(1,1:end))
hold on
plot(x2(1,1:end))
hold on

legend("vc","v","v_{ref}")
ylabel('x [m/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of ul Errors}$','Interpreter','latex','FontSize',9);

figure(5)
plot(uc(2,1:end))
hold on
plot(x2(2,1:end))
hold on

legend("vc","v","v_{ref}")
ylabel('y [m/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of um Errors}$','Interpreter','latex','FontSize',9);


% %%%%%%%%%%%%%
figure(8)
plot(chi_estimados(:,:)')

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(sample)),sample,'Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'$t_{sample}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Sample Time}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);