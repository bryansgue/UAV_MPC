% Programa de control predictivo para un drone basado en optimizacion usando Casadi

% Eliminar variables del sistema
clear all;
close all;
clc;

%Generacion de los tiempos del sistema
f = 30 % Hz 
ts = 1/f; 
tfinal = 5;
t = [0:ts:tfinal];
 
% Definicion del horizonte de prediccion
N = 1.5*f; 

% Definicion de las constantes del sistema
a = 0.0;
b = 0.0;
L =[a;b];

% Definicion de los limites de las acciondes de control
bounded = 3*[1.2; -1.2; 1.2; -1.2; 1.2; -1.2; 5.5; -5.5];

% Seccion para cargar los parametros dinamicos del sistema

load("chi_values.mat");

% Deficion de la matriz de la matriz de control
Q = 1.5*eye(4);

% Definicion de la matriz de las acciones de control
R = 0.01*eye(4);

% Definicion de los estados iniciales del sistema
x = 0;
y = 0;
z = 0;
psi(1,1) = (180*pi/180);

%% a) Posiciones iniciales del UAV
hx(1) = 0; 
hy(1) = 0; 
hz(1) = 1; 
psi(1)= 0;
h=[hx(1);hy(1);hz(1);psi(1)]

% Definicion del vectro de control inicial del sistema
v_N = zeros(N,4);
H0 = repmat([hx(1);hy(1);hz(1);psi(1)],1,N+1)';
x_N = H0;

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[hxd, hyd, hzd, psid, hxdp, hydp, hzdp, psidp] = Trayectorias(3,t,5);
%% GENERALIZED DESIRED SIGNALS
psid = 0*psid;
hd = [hxd; hyd; hzd; psid];
hdp = [hxdp;hydp;hzdp;psidp];

%% Velocidad inicial real del UAV
u_real = [0;0;0;0];
chi_estimados(:,1) = chi';
chi_real(:,1) = chi';
%% Ganancia Compensacion Dinamica
K1 = 1;
K2 = 1;

%% Definicion del optimizador
[f, solver, args] = mpc_droneKin(bounded, N, L, ts, Q, R);

tic
for k=1:length(t)-N   
    %% Generacion del; vector de error del sistema
    he(:,k)=hd(:,k)-h(:,k);
       
    tic
    [u_opt,x_opt] = SolverUAV_MPC(h,hd,N,x_N,v_N,args,solver,k);
    sample(k)=toc;
    
    vc(:,k)= u_opt(1,:)';
    h_N(:,1:4,k) = x_opt;
    
   %% Compensador Dinamico
    [vref(:,k),chi_estimados(:,k+1)] = adaptive_OPTI(0*vc(:,k), vc(:,k), u_real(:,k), chi_estimados(:,k),chi, K1, K2, L, ts);
    
    %% Dinamica del sistema 
    u_real(:, k+1) = dyn_model_adapUAV(chi_real, u_real(:,k), vref(:,k), psi(k), L,ts,k);
    %v_estimate(:, k+1) = dynamic_model_for_sim(x, v_estimate(:,k), v_ref(:,k), psi(k), L,ts);
    %v_estimate(:, k+1) = dynamic_model_for_sim(x, v_estimate(:,k), v_ref(:,k), psi(k), L,ts);
    
    h(:,k+1) = h(:,k)+ UAV_RK4(h(:,k),u_real(:,k+1),ts);
    hx(k+1) = h(1,k+1);
    hy(k+1) = h(2,k+1);
    hz(k+1) = h(3,k+1);      
    psi(k+1) = (h(4,k+1));
        
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
h = light;
h.Color=[0.65,0.65,0.65];
h.Style = 'infinite';
%b) Dimenciones del Robot
   Drone_Parameters(0.02);
%c) Dibujo del Robot    
    G2=Drone_Plot_3D(hx(1),hy(1),hz(1),0,0,psi(1));hold on

    G3 = plot3(hx(1),hy(1),hz(1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   
    G4 = plot3(hxd(1),hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);
    G5 = Drone_Plot_3D(hx(1),hy(1),hz(1),0,0,psi(1));hold on
    %plot3(hxd(ubicacion),hyd(ubicacion),hzd(ubicacion),'*r','linewidth',1.5);
    view(20,15);

    paso = 5;
for k = 1:paso:length(t)-N
    drawnow
    delete(G2);
    delete(G3);
    delete(G4);
    delete(G5);
    G2=Drone_Plot_3D(hx(k),hy(k),hz(k),0,0,psi(k));hold on
    G3 = plot3(hxd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    G4 = plot3(hx(1:k),hy(1:k),hz(1:k),'-.','Color',[56,171,217]/255,'linewidth',1.5);
    G5 = plot3(h_N(1:N,1,k),h_N(1:N,2,k),h_N(1:N,3,k),'Color',[100,100,100]/255,'linewidth',0.1);
    %if k<10*paso
     %   pause(1)
    %end
    
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
plot(t(1:length(he)),he(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(he)),he(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid on;
legend({'$\tilde{h_{x}}$','$\tilde{h_{y}}$','$\tilde{h_{z}}$','$\tilde{h_{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);


% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aÃ©reo
figure(3)

subplot(4,1,1)
plot(hxd)
hold on
plot(hx)
legend("xd","hx")
ylabel('x [m]'); xlabel('s [ms]');
title('$\textrm{Evolution of h }$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(hyd)
hold on
plot(hy)
legend("yd","hy")
ylabel('y [m]'); xlabel('s [ms]');


subplot(4,1,3)
plot(hzd)
hold on
plot(hz)
grid on
legend("zd","hz")
ylabel('z [m]'); xlabel('s [ms]');

subplot(4,1,4)
plot(Angulo(psid))
hold on
plot(psi)
legend("psid","psi")
ylabel('psi [rad]'); xlabel('s [ms]');


% %%%%%%%%%%%%%
figure(4)

plot(vc(1,1:end))
hold on
plot(u_real(1,1:end))
hold on
plot(vref(1,1:end))
legend("vc","v","v_{ref}")
ylabel('x [m/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of ul Errors}$','Interpreter','latex','FontSize',9);

figure(5)
plot(vc(2,1:end))
hold on
plot(u_real(2,1:end))
hold on
plot(vref(2,1:end))
legend("vc","v","v_{ref}")
ylabel('y [m/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of um Errors}$','Interpreter','latex','FontSize',9);

figure(6)
plot(vc(3,1:end))
hold on
plot(u_real(3,1:end))
hold on
plot(vref(3,1:end))
legend("vc","v","v_{ref}")
ylabel('z [m/ms]'); xlabel('s [ms]');
title('$\textrm{Evolution of un Errors}$','Interpreter','latex','FontSize',9);

figure(7)
plot(vc(4,1:end))
hold on
plot(u_real(4,1:end))
hold on
plot(vref(4,1:end))
legend("vc","v","v_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of w Errors}$','Interpreter','latex','FontSize',9);

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