% Programa de control predictivo para un drone basado en optimizacion usando Casadi
%% Clear variables
clc, clear all, close all;

load("chi_simple.mat");
chi_uav = chi';

%% DEFINITION OF TIME VARIABLES
f = 30 % Hz 
ts = 1/f;
to = 0;
tf = 30;
t = (to:ts:tf);


%% Inicializa Nodo ROS
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.88.86:11311')
setenv('ROS_IP','192.168.88.104')
rosinit

%% 1) PUBLISHER TOPICS & MSG ROS - UAV M100
[velControl_topic,velControl_msg] = rospublisher('/m100/velocityControl','geometry_msgs/TwistStamped');
u_ref(:,1) = [0.0, 0.0, 0.0, 0.0];
send_velocities(velControl_topic, velControl_msg, u_ref(:,1));

%% 2) Suscriber TOPICS & MSG ROS - UAV M100
odomSub = rossubscriber('/dji_sdk/odometry');

[pose(:,1),euler(:,1),v(:,1),omega(:,1),quat(:,1)] = odometryUAV(odomSub);

euler_p(:,1) = [0;0;0];

%% Definicion del horizonte de prediccion
N = f; 

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
h = [0;0;1;psi]

%% INITIAL GENERALIZE VELOCITIES
u = [0; 0;0;0];
x = [h;u]; 
%% GENERAL VECTOR DEFINITION
H = [h;u];

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[hxd, hyd, hzd, hpsid, hxdp, hydp, hzdp, hpsidp] = Trayectorias(3,t,12);

%% GENERALIZED DESIRED SIGNALS
%hd = [hxd; hyd; hzd; hpsid];
hd = [hxd;hyd;hzd;0*hpsid;hxdp; hydp; hzdp; 0*hpsidp];

%hdp = [hxdp;hydp;hzdp;hpsidp];

%% Deficion de la matriz de la matriz de control
Q = 1.5*eye(4);

%% Definicion de la matriz de las acciones de control
K = 0.5*eye(4);

%% Definicion de los limites de las acciondes de control
bounded = 3*[1.2; -1.2; 1.2; -1.2; 1.2; -1.2; 5.5; -5.5];

%% Definicion del vectro de control inicial del sistema
v_N = zeros(N,4);
H0 = repmat(H,1,N+1)'; 
x_N = H0;

% Definicion del optimizador
[f, solver, args] = mpc_drone_estimator(chi_uav,bounded, N, L, ts, Q, K);
A = 1;
B = 0.01;
% Chi estimado iniciales
chi_estimados(:,1) = chi';

% External torque of the system
v_extern = zeros(4, length(t));

% Aux time variable 
% Aux time variable 
t_aux = (t >= 5) & (t < 10);
v_extern(1, t_aux) = 1.7;
v_extern(2, t_aux) = -1.2;
v_extern(3, t_aux) = 1.8;
v_extern(4, t_aux) = -1.0;

t_aux_1 = (t >= 13) & (t < 19);
v_extern(1, t_aux_1) = 1.8;
v_extern(2, t_aux_1) = -1.5;
v_extern(3, t_aux_1) = 1.1;
v_extern(4, t_aux_1) = -1.9;

t_aux_2 = (t >= 23) & (t < 28);
v_extern(1, t_aux_2) = -1.8;
v_extern(2, t_aux_2) = 1.0;
v_extern(3, t_aux_2) = -1.5;
v_extern(4, t_aux_2) = 1.9;


%% Inicializa el estimador
Tu_est(:,1) = [0;0;0;0];
Gain = 6;

tic
for k=1:length(t)-N

    %% Generacion del; vector de error del sistema
    he(:,k)=hd(1:4,k)-h(:,k);
    
    %%
    tic
    [u_opt,x_opt] = SolverUAV_MPC_din_estimator(h,u,hd,N,x_N,v_N,args,solver,k,0*Tu_est(:,k));
    sample(k)=toc;
    
    uc(:,k)= u_opt(1,:)';
    h_N(:,1:4,k) = x_opt(:,1:4);
    
    %% Aceleracion
    if k>1
    uc_p(:,k)=(uc(:,k)- uc(:,k-1))/ts;
    else
    uc_p(:,k) = [0;0;0;0];    
    end
    %vcp(:,k) = [ulp(k);ump(k);unp(k);wp(k)];
     uc_p(:,k) = [0;0;0;0];
    
    %% DYNAMIC ESTIMATION
    [Test(:,k),chi_estimados(:,k+1)] = estimadaptive_dymanic_UAV(chi_estimados(:,k),uc_p(:,k), uc(:,k), u(:,k), hd(1:4,k), h(:,k) ,A,B, L, ts);
    u_ref(:,k)= uc(:,k)+Test(:,k)-Tu_est(:,k);
     
    %% Dinamica del sistema 


    send_velocities(velControl_topic, velControl_msg, u_ref(:,k));
    
    %% 3) Odometria del UAV
    
    [pose(:,k+1),euler(:,k+1),v(:,k+1),omega(:,k+1),quat(:,k+1)] = odometryUAV(odomSub);
    
    euler_p(:,k+1) = Euler_p(omega(:,k+1),euler(:,k+1));
      
    R(:,:,k) = QuatToRot(quat(:,k+1));

    s(:,k+1) = inv(R(:,:,k))*v(:,k+1);
    
        
    h(:,k+1) = [pose(:,k+1);euler(3,k)];
    u(:,k+1) = [s(:,k+1);euler_p(3,k)];
    
    %%
    
        %% Observador
    if k>1
        u_p(:,k) = (u(:,k)- u(:,k-1))/ts ;
    else
        u_p(:,k) = [0;0;0;0];
    end

    Tu_est(:,k+1) = Momentum_Observer(chi_uav,u_ref(:,k),u(:,k),u_p(:,k),Tu_est(:,k),L,ts,Gain);
        
    %% Actualizacion de los resultados del optimizador para tener una soluciona aproximada a la optima
     
%   u_opt(2,:) = u_opt(2,:)+Test(:,k)';
    v_N = [u_opt(2:end,:);u_opt(end,:)];
    x_N = [x_opt(2:end,:);x_opt(end,:)];
end

%% 1) PUBLISHER TOPICS & MSG ROS
u_ref = [0.0, 0, 0, 0];
send_velocities(velControl_topic, velControl_msg, u_ref);

toc
%%
% close all; paso=1; 
% %a) Parámetros del cuadro de animación
% figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPositionMode', 'manual');
%     set(gcf, 'PaperPosition', [0 0 8 3]);
%     luz = light;
%     luz.Color=[0.65,0.65,0.65];
%     luz.Style = 'infinite';
%b) Dimenciones del Robot
    Drone_Parameters(0.02);
%c) Dibujo del Robot    
    G2=Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on

    G3 = plot3(h(1,1),h(2,1),h(3,1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   
    G4 = plot3(hxd(1),hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);
    G5 = Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on
%    plot3(hxd(ubicacion),hyd(ubicacion),hzd(ubicacion),'*r','linewidth',1.5);
    view(-45,45);
    
%     G6=Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on

    G7 = plot3(h(1,1),h(2,1),h(3,1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   
    G8 = plot3(hxd(1),hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);
%     G9 = Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on
% 
for k = 1:10:length(t)-N
    drawnow
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

%%
figure;

% Subplot 1
subplot(4,1,1)

plot(Tu_est(1,:), 'LineWidth', 2, 'DisplayName', 'Tx_{est}')
grid on
ylabel('x [m]', 'FontSize', 10);
title('Evolution of x', 'FontSize', 12);
legend('Location', 'best');

% Subplot 2
subplot(4,1,2)

plot(Tu_est(2,:), 'LineWidth', 2, 'DisplayName', 'Ty_{est}')
grid on
ylabel('y [m]', 'FontSize', 10);
title('Evolution of y', 'FontSize', 12);
legend('Location', 'best');

% Subplot 3
subplot(4,1,3)

plot(Tu_est(3,:), 'LineWidth', 2, 'DisplayName', 'Tz_{est}')
grid on
ylabel('z [m]', 'FontSize', 10);
title('Evolution of z', 'FontSize', 12);
legend('Location', 'best');

% Subplot 4
subplot(4,1,4)

plot(Tu_est(4,:), 'LineWidth', 2, 'DisplayName', 'Tpsi_{est}')
grid on
ylabel('psi [rad]', 'FontSize', 10);
xlabel('Time [kT_0]', 'Interpreter', 'latex', 'FontSize', 10);
legend('Location', 'best');
%% %%%%%%%%%%%%%

figure(5)

plot(uc(1,1:end))
hold on
plot(u(1,1:end))
hold on
plot(u_ref(1,1:end))
legend("ul_c","ul","ul_{ref}")
ylabel('x [m/s]'); xlabel('s [ms]');
%title('$\textrm{Evolution of ul Errors}$','Interpreter','latex','FontSize',9);

figure(6)
plot(uc(2,1:end))
hold on
plot(u(2,1:end))
hold on
plot(u_ref(2,1:end))
legend("um_c","um","um_{ref}")
ylabel('y [m/s]'); xlabel('s [ms]');
title('$\textrm{Evolution of um Errors}$','Interpreter','latex','FontSize',9);

figure(7)
plot(uc(3,1:end))
hold on
plot(u(3,1:end))
hold on
plot(u_ref(3,1:end))
legend("un_c","un","un_{ref}")
ylabel('z [m/ms]'); xlabel('s [ms]');
title('$\textrm{Evolution of un Errors}$','Interpreter','latex','FontSize',9);

figure(8)
plot(uc(4,1:end))
hold on
plot(u(4,1:end))
hold on
plot(u_ref(4,1:end))
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