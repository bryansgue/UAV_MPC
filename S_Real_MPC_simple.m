% Programa de control predictivo para un drone basado en optimizacion usando Casadi
%% Clear variables
clc, clear all, close all;

load("chi_simple.mat");
chi_real = chi';

%% DEFINITION OF TIME VARIABLES
f = 30 % Hz 
ts = 1/f;
to = 0;
tf = 15;
t = (to:ts:tf);


%% Inicializa Nodo ROS
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.88.86:11311')
setenv('ROS_IP','192.168.88.104')
rosinit

%% Definicion del horizonte de prediccion
N = f/2; %20 

%% CONSTANTS VALUES OF THE ROBOT
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

% Definicion de los estados iniciales del sistema
hx(1) = 0;
hy(1) = 0;
hz(1) = 0;
psi(1) = 0;
h = [hx;hy;hz;psi]

%% INITIAL GENERALIZE VELOCITIES
u = [0; 0;0;0];

h = [0;0;1;0];

%% INITIAL GENERALIZE VELOCITIES
u = [0; 0;0;0];
x = [h;u];

%% GENERAL VECTOR DEFINITION
H = [h;u];

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[hxd, hyd, hzd, hpsid, ul_d, um_d, un_d, r_d] = Trayectorias(3,t,5);

%% GENERALIZED DESIRED SIGNALS
%hd = [hxd; hyd; hzd; hpsid];
hd = [hxd;hyd;hzd;0*hpsid; ul_d; um_d; un_d; 0*r_d];
% ud = [hxdp; hydp; hzdp; 0*hpsidp]
%hdp = [hxdp;hydp;hzdp;hpsidp];

%% Deficion de la matriz de la matriz de control
Q = 1*eye(4);

%% Definicion de la matriz de las acciones de control
R = 0.001*eye(4);

%% Definicion de los limites de las acciondes de control
bounded = 3*[1; -1; 1; -1; 1; -1; 1; -1];

%% Definicion del vectro de control inicial del sistema

v_N = zeros(N,4);
H0 = repmat(H,1,N+1)'; 
x_N = H0;

% Definicion del optimizador
[f, solver, args] = S_mpc_drone(chi_real,bounded, N, L, ts, Q, R);



RcSub = rossubscriber('/dji_sdk/rc');
hd = RC_subscriber(RcSub);

for k=1:length(t)-N

    %% Generacion del; vector de error del sistema
    he(:,k)=hd(1:4,k)-h(:,k);
    
    tic
    [u_opt,x_opt] = SolverUAV_MPC_din(h,u,hd,N,x_N,v_N,args,solver,k);
    sample(k)=toc;
    
    vref(:,k)= u_opt(1,:)';
    h_N(:,1:4,k) = x_opt(:,1:4);


    %% DYNAMIC ESTIMATION
      
    x(:,k+1) = UAV_Dinamica_RK4(chi_real,x(:,k),vref(:,k),L,ts);
    
    h(:,k+1) = x(1:4,k+1);
    u(:,k+1) = x(5:8,k+1);
     
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
    luz = light;
    luz.Color=[0.65,0.65,0.65];
    luz.Style = 'infinite';
%b) Dimenciones del Robot
    Drone_Parameters(0.02);
%c) Dibujo del Robot    
    G2=Drone_Plot_3D(hx(1),hy(1),hz(1),0,0,psi(1));hold on

    G3 = plot3(hx(1),hy(1),hz(1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   
    G4 = plot3(hxd(1),hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);
    G5 = Drone_Plot_3D(hx(1),hy(1),hz(1),0,0,psi(1));hold on
%    plot3(hxd(ubicacion),hyd(ubicacion),hzd(ubicacion),'*r','linewidth',1.5);
    view(20,15);
    


for k = 1:10:length(t)-N
    %drawnow
    delete(G2);
    delete(G3);
    delete(G4);
    delete(G5);
   
    G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),0,0,h(4,k));hold on  
    G3 = plot3(hxd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    G4 = plot3(h(1,1:k),h(2,1:k),h(3,1:k),'-.','Color',[56,171,217]/255,'linewidth',1.5);
    G5 = plot3(h_N(1:N,1,k),h_N(1:N,2,k),h_N(1:N,3,k),'Color',[100,100,100]/255,'linewidth',0.1);

    pause(0)
end
%%



%%
figure(5)

plot(vref(1,1:end), 'LineWidth', 2, 'DisplayName', 'ul_{ref}')
hold on
plot(u(1,1:end), 'LineWidth', 2, 'DisplayName', 'ul_{real}')
hold on
plot(ul_d(1,1:end), 'LineWidth', 2, 'DisplayName', 'ul_{d}')
legend()
ylabel('x [m/s]'); xlabel('s [ms]');
%title('$\textrm{Evolution of ul Errors}$','Interpreter','latex','FontSize',9);
%%
figure(6)
plot(vref(2,1:end), 'LineWidth', 2, 'DisplayName', 'um_{ref}')
hold on
plot(u(2,1:end), 'LineWidth', 2, 'DisplayName', 'um_{real}')
hold on
plot(um_d(1,1:end), 'LineWidth', 2, 'DisplayName', 'um_{d}')
legend()
ylabel('y [m/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of um Errors}$','Interpreter','latex','FontSize',9);
%%
figure(7)
plot(vref(3,1:end), 'LineWidth', 2, 'DisplayName', 'un_{ref}')
hold on
plot(u(3,1:end), 'LineWidth', 2, 'DisplayName', 'un_{real}')
hold on
plot(un_d(1,1:end), 'LineWidth', 2, 'DisplayName', 'un_{d}')
legend()
ylabel('z [m/ms]'); xlabel('s [ms]');
title('$\textrm{Evolution of un Errors}$','Interpreter','latex','FontSize',9);
%%
figure(8)
plot(vref(4,1:end))
hold on
plot(u(4,1:end))
hold on
plot(0*r_d(1,1:end))
legend("w_c","w","w_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of w Errors}$','Interpreter','latex','FontSize',9);

