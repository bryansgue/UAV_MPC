function x_p = f_DMD(x, vref, L,A,B)

a = L(1);
b = L(2);
c = L(3);

psi = x(4);

J = [cos(psi), -sin(psi), 0, -(a*sin(psi)+b*cos(psi));...
     sin(psi), cos(psi), 0,   (a*cos(psi)-b*sin(psi));...
     0, 0, 1, 0;...
     0, 0, 0, 1]; 

%% Definicion del Sistemaa
M = [zeros(4,4),J;...
     zeros(4,4),A];

C = [zeros(4,4);
     B];

%% vector que incluye el vector de estados y la referencia

x_p= M * x + C * vref;

end