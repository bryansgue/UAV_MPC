% Programa de control predictivo para un drone basado en optimizacion usando Casadi
%% Clear variables
clc, clear all, close all;

load("chi_values.mat");
chi_real = chi';

%% DEFINITION OF TIME VARIABLES
f = 30 % Hz 
ts = 1/f;
to = 0;
tf = 15;
t = (to:ts:tf);

%% Definicion del horizonte de prediccion
N = f/2; %20 

%% CONSTANTS VALUES OF THE ROBOT
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

% Definicion de los estados iniciales del sistema
x(1) = 0;
y(1) = 0;
z(1) = 0;
psi(1) = 0;
h = [x;y;z;psi]

%% INITIAL GENERALIZE VELOCITIES
v = [0; 0;0;0];

%% GENERAL VECTOR DEFINITION
H = [h;v];

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[hxd, hyd, hzd, hpsid, hxdp, hydp, hzdp, hpsidp] = Trayectorias(3,t,5);

%% GENERALIZED DESIRED SIGNALS
%hd = [hxd; hyd; hzd; hpsid];
hd = [hxd;hyd;hzd;0*hpsid;hxdp; hydp; hzdp; 0*hpsidp];

%hdp = [hxdp;hydp;hzdp;hpsidp];

%% Deficion de la matriz de la matriz de control
Q = 1*eye(4);

%% Definicion de la matriz de las acciones de control
R = 0.001*eye(4);

%% Definicion de los limites de las acciondes de control
bounded = 3*[1.2; -1.2; 1.2; -1.2; 1.2; -1.2; 5.5; -5.5];

%% Definicion del vectro de control inicial del sistema

v_N = zeros(N,4);
H0 = repmat(H,1,N+1)'; 
x_N = H0;

% Definicion del optimizador
[f, solver, args] = mpc_drone(chi_real,bounded, N, L, ts, Q, R);

% Chi estimado iniciales
chi_estimados(:,1) = chi';
tic
for k=1:length(t)-N

    %% Generacion del; vector de error del sistema
    he(:,k)=hd(1:4,k)-h(:,k);
    
    tic
    [u_opt,x_opt] = SolverUAV_MPC_din(h,v,hd,N,x_N,v_N,args,solver,k);
    sample(k)=toc;
    
    vc(:,k)= u_opt(1,:)';
    h_N(:,1:4,k) = x_opt(:,1:4);

    %% 2) ACELERATIONS VCP 
    if k>1
    ulp(k)=(vc(1,k)- vc(1,k-1))/ts;
    ump(k)=(vc(2,k)- vc(2,k-1))/ts;
    unp(k)=(vc(3,k)- vc(3,k-1))/ts;
    wp(k) =(vc(4,k)- vc(4,k-1))/ts;
    else
    ulp(k)=0;   
    ump(k)=0; 
    unp(k)=0; 
    wp(k) =0; 
    end
    %vcp(:,k) = [ulp(k);ump(k);unp(k);wp(k)];
    vcp(:,k) = [0;0;0;0];
    %% DYNAMIC ESTIMATION
    
    %% Dinamica del sistema 
    [v(:, k+1),Tu(:,k)] = model_Dyn_UAV(chi_real, v(:,k), vc(:,k), psi(k), L,ts,k);
    
    %% Simulacion del sistema
%     h=h+system(h,[ul(k+1);um(k+1);un(k+1);w(k+1)],f,ts);
    
    h(:,k+1) = h(:,k)+ UAV_RK4(h(:,k),v(:,k+1),ts);
    hx(k+1) = h(1,k+1);
    hy(k+1) = h(2,k+1);
    hz(k+1) = h(3,k+1);      
    psi(k+1) = Angulo(h(4,k+1));
    
        
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
%    plot3(hxd(ubicacion),hyd(ubicacion),hzd(ubicacion),'*r','linewidth',1.5);
    view(20,15);
    


for k = 1:10:length(t)-N
    %drawnow
    delete(G2);
    delete(G3);
    delete(G4);
    delete(G5);
   
    G2=Drone_Plot_3D(hx(k),hy(k),hz(k),0,0,psi(k));hold on  
    G3 = plot3(hxd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    G4 = plot3(hx(1:k),hy(1:k),hz(1:k),'-.','Color',[56,171,217]/255,'linewidth',1.5);
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


figure

subplot(4,1,1)
plot(Tu(1,:))
hold on
plot(Test(1,:))
legend("Tx_u","Tx_{est}")
ylabel('x [m]');
%title('$\textrm{Evolution of h }$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(Tu(2,:))
hold on
plot(Test(2,:))
legend("Ty_u","Ty_{est}")
ylabel('y [m]'); 

subplot(4,1,3)
plot(Tu(3,:))
hold on
plot(Test(3,:))
grid on
legend("Tz_u","Tz_{est}")
ylabel('z [m]'); 

subplot(4,1,4)
plot(Tu(4,:))
hold on
plot(Test(4,:))
legend("Tpsi_u","Tpsi_{est}")
ylabel('psi [rad]'); 
xlabel('$\textrm{Time }[kT_0]$','Interpreter','latex','FontSize',9);
% %%%%%%%%%%%%%

figure(5)

plot(vc(1,1:end))
hold on
plot(v(1,1:end))
hold on
plot(vref(1,1:end))
legend("ul_c","ul","ul_{ref}")
ylabel('x [m/s]'); xlabel('s [ms]');
%title('$\textrm{Evolution of ul Errors}$','Interpreter','latex','FontSize',9);

figure(6)
plot(vc(2,1:end))
hold on
plot(v(2,1:end))
hold on
plot(vref(2,1:end))
legend("um_c","um","um_{ref}")
ylabel('y [m/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of um Errors}$','Interpreter','latex','FontSize',9);

figure(7)
plot(vc(3,1:end))
hold on
plot(v(3,1:end))
hold on
plot(vref(3,1:end))
legend("un_c","un","un_{ref}")
ylabel('z [m/ms]'); xlabel('s [ms]');
title('$\textrm{Evolution of un Errors}$','Interpreter','latex','FontSize',9);

figure(8)
plot(vc(4,1:end))
hold on
plot(v(4,1:end))
hold on
plot(vref(4,1:end))
legend("w_c","w","w_{ref}")
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