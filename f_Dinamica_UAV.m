function x_p = f_Dinamica_UAV(x, vref, L, chi_uav)

a = L(1);
b = L(2);
c = L(3);

psi = x(4);
w = x(8);

J = [cos(psi), -sin(psi), 0, -(a*sin(psi)+b*cos(psi));...
     sin(psi), cos(psi), 0,   (a*cos(psi)-b*sin(psi));...
     0, 0, 1, 0;...
     0, 0, 0, 1]; 
 
M11=chi_uav(1);
M12=0;
M13=0;
M14=b*chi_uav(2);
M21=0;
M22=chi_uav(3) ;
M23=0;
M24=a*chi_uav(4);
M31=0;
M32=0;
M33=chi_uav(5);
M34=0;
M41=b*chi_uav(6);
M42=a*chi_uav(7);
M43=0;
M44=chi_uav(8)*(a^2 + b^2) + chi_uav(9);

M=[M11,M12,M13,M14;...
    M21,M22,M23,M24;...
    M31,M32,M33,M34;...
    M41,M42,M43,M44];

%% CENTRIOLIS MATRIX
C11=chi_uav(10);
C12=w*chi_uav(11);
C13=0;
C14=a*w*chi_uav(12);
C21=w*chi_uav(13);
C22=chi_uav(14);
C23=0;
C24=b*w*chi_uav(15);
C31=0;
C32=0;
C33=chi_uav(16);
C34=0;
C41=a*w*chi_uav(17);
C42=b*w*chi_uav(18);
C43=0;
C44=chi_uav(19);

C=[C11,C12,C13,C14;...
    C21,C22,C23,C24;...
    C31,C32,C33,C34;...
    C41,C42,C43,C44];

%% GRAVITATIONAL MATRIX
G11=0;
G21=0;
G31=0;
G41=0;

G=[G11;G21;G31;G41];
%% Definicion del Sistemaa

A = [zeros(4,4),J;...
     zeros(4,4),-inv(M)*C];

B = [zeros(4,4);
     inv(M)];
    
aux = [zeros(4,1);...
       -inv(M)*G]; 

%% vector que incluye el vector de estados y la referencia

x_p= A * x + B * vref + aux;

end