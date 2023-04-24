%% Vertical dynamics drone

%% clean variables 
clc, clear all, close all;

%% Time definition

t_s = 0.01;
t_final = 5;
t = (0:t_s:t_final);
%% Intitial conditions position
x = 0;
y = 0;
z = 0;

h = [x;y;z];
%% Initial condition definition velocities
vx = 0;
vy = 0;
vz = 0;

Vb = [vx;vy;vz];

%% Initial conditions angles
phi = 0*(pi/180);
theta = 0;
psi = 0*(pi/180);

angles_b = [phi;theta;psi];

%% Rotational Matrix
R_NC = Rot_zyx(angles_b);
R_total = zeros(3,3,length(t)+1);
R_total(:,:,1) = R_NC;

%% Angular velocities 
omega = [0; 0; 0];

H = [h;Vb];

%% System parameters
g = 9.8;
m = 1.25;

L = [g;m];

%% Inertial Value
I = [0.0232, 0, 0;...
     0, 0.0232, 0;...
     0 0 0.0468];

%% Desired reference z

zd = 0.5*ones(1,length(t));%1*cos(0.5*t);
zdp = 0*ones(1,length(t));
k1 = 0.3;

%% Control values torques
T = [0.0001*cos(t);...
     0.000*cos(t);...
     0.000*cos(t)];

for k = 1:1:length(t)
    %% Error definition
    
    %% Control Law

    F = 1*g*m;
    
    %% System evolution
    H(:,k+1) = system_linear_dynamics(H(:,k), R_total(:,:,k), F, L, t_s);
    R_total(:,:, k+1) = system_angular_dynamics_r(R_total(:, :, k), omega(:, k), t_s);
    angles_b(:, k+1) = system_angular_dynamics_euler(angles_b(:, k), omega(:,k), t_s);
    omega(:, k+1) = system_angular_dynamics(omega(:, k), T(:, k), I, t_s);
    
end


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
    G2=Drone_Plot_3D(H(1,1),H(2,1),H(3,1),angles_b(1,1), angles_b(2,1), angles_b(3,1));hold on

    plot3(H(1,1),H(2,1),H(3,11),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   


view(20,15);
for k = 1:10:length(t)-1
    drawnow
    delete(G2);
   
    G2=Drone_Plot_3D(H(1,k),H(2,k),H(3,k),angles_b(1, k), angles_b(2, k), angles_b(3, k));hold on
    
    plot3(H(1,1:k),H(2,1:k),H(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    %plot3(obs(1,:),obs(2,:),obs(3,:),'x','Color',[0,171,217]/255,'linewidth',2);
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end


%% System pictures
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
%plot(t(1:length(he)),he(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
grid('minor')
grid on;
legend({'$\tilde{h_{z}}$','$\tilde{h_{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,H(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,H(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,H(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on

grid on;
legend({'$x$','$y$','$z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,angles_b(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,angles_b(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,angles_b(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on

grid on;
legend({'$\phi$','$\theta$','$\psi$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
%legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$','$\mu_{l}$','$\mu_{m}$','$\mu_{n}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,H(4,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,H(5,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,H(6,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,zd,'--','Color',[26,115,160]/255,'linewidth',1); hold on

grid on;
legend({'$v_x$','$v_y$','$v_z$','$v_{zd}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
