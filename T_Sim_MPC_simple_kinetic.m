% Programa de control predictivo para un drone basado en optimizacion usando Casadi
%% Clear variables
clc, clear all, close all;

%% DEFINITION OF TIME VARIABLES
f = 30; % Hz 
ts = 1/f;
to = 0;
tf = 30;
t = (to:ts:tf);

%% Definicion del horizonte de prediccion
N = f; %20 

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
h = [hx;hy;hz;psi];

%% GENERAL VECTOR DEFINITION
H = h;

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[hxd, hyd, hzd, hpsid, hxdp, hydp, hzdp, hpsidp] = Trayectorias(3,t,5);

%% GENERALIZED DESIRED SIGNALS

hd = [hxd;hyd;hzd;0*hpsid;hxdp; hydp; hzdp; 0*hpsidp];

%% Deficion de la matriz de la matriz de control
Q = 1.5*eye(4);

%% Definicion de la matriz de las acciones de control
R = 0.001*eye(4);

%% Definicion de los limites de las acciondes de control
bounded = 3*[1.2; -1.2; 1.2; -1.2; 1.2; -1.2; 0.5; -0.5];

%% Definicion del vectro de control inicial del sistema

v_N = zeros(N,4);
H0 = repmat(H,1,N+1)'; 
x_N = H0;

% Definicion del optimizador
[f, solver, args] = mpc_drone_kinetic(bounded, N, L, ts, Q, R);

tic
for k=1:length(t)-N

    %% Generacion del; vector de error del sistema
    he(:,k)=hd(1:4,k)-h(:,k);
    
    tic
    [u_opt,x_opt] = SolverUAV_MPC_kin(h,hd,N,x_N,v_N,args,solver,k);
    sample(k)=toc;
    disp(toc)
    
    u_ref(:,k)= u_opt(1,:)';
    h_N(:,1:4,k) = x_opt(:,1:4);

    %% DYNAMIC ESTIMATION
    
    psi = h(4,k);
    J = [cos(psi) -sin(psi) 0 0;
         sin(psi) cos(psi) 0 0;
         0 0 1 0;
         0 0 0 1];
     
    h_p(:,k) = J*u_ref(:,k);
    
%% 3) Recibir datos - UAV       
     h(:,k+1) = ts*h_p(:,k)+h(:,k);

     
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

legend({'$\tilde{\eta}_{x}$','$\tilde{\eta}_{y}$','$\tilde{\eta}_{z}$','$\tilde{\eta}_{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');0
legend('boxoff')
%title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time }[s]$','Interpreter','latex','FontSize',9);
% xlabel('$Time[s]$','Interpreter','latex','FontSize',9);


%%
figure(5)

plot(hd(1,1:end))
hold on
plot(h(1,1:end))

legend({'${\eta}_{x_d}$','${\eta}_{x}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
ylabel('x [m/s]'); xlabel('s [ms]');
%title('$\textrm{Evolution of ul Errors}$','Interpreter','latex','FontSize',9);

%%
figure(6)
plot(hd(2,1:end))
hold on
plot(h(2,1:end))

legend({'${\eta}_{x_d}$','${\eta}_{x}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
ylabel('y [m/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of um Errors}$','Interpreter','latex','FontSize',9);

%%

figure(7)
plot(hd(3,1:end))
hold on
plot(h(3,1:end))

legend({'${\eta}_{z_d}$','${\eta}_{z}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
ylabel('z [m/ms]'); xlabel('s [ms]');
title('$\textrm{Evolution of un Errors}$','Interpreter','latex','FontSize',9);

%%

figure(8)
plot(hd(4,1:end))
hold on
plot(h(4,1:end))

legend({'${\eta}_{\psi_d}$','${\eta}_{x}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');

ylabel('psi [rad/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of w Errors}$','Interpreter','latex','FontSize',9);


% figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 10 4]);
% plot(t(1:length(ul_c)),ul_c,'Color',[226,76,44]/255,'linewidth',1); hold on
% plot(t(1:length(ul_c)),um_c,'Color',[46,188,89]/255,'linewidth',1); hold on
% plot(t(1:length(ul_c)),un_c,'Color',[26,115,160]/255,'linewidth',1); hold on
% plot(t(1:length(ul_c)),w_c,'Color',[83,57,217]/255,'linewidth',1); hold on
% plot(t(1:length(ul)),ul,'--','Color',[226,76,44]/255,'linewidth',1); hold on
% plot(t(1:length(ul)),um,'--','Color',[46,188,89]/255,'linewidth',1); hold on
% plot(t(1:length(ul)),un,'--','Color',[26,115,160]/255,'linewidth',1); hold on
% plot(t(1:length(ul)),w,'--','Color',[83,57,217]/255,'linewidth',1); hold on
% grid on;
% legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$','$\mu_{l}$','$\mu_{m}$','$\mu_{n}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend('boxoff')
% title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
% ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
% xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(sample)),sample,'Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'$t_{sample}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%title('$\textrm{Sample Time}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time }[kT_0]$','Interpreter','latex','FontSize',9);
%xlabel('$Time[kT_0]$','Interpreter','latex','FontSize',9);