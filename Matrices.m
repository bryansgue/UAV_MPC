function [A1,B1,Q1,G1,A2,B2,C2,D2,E2,G_tras ] = Matrices(chi)
g = 9.81;
m = chi(25);

A1 = [0 0 0;
    0 0 0
    0 0 chi(26)];
B1 = [0 0 0;
    0 0 0
    0 0 chi(27)];
Q1 = [0 0 0;
    0 0 0
    0 0 chi(28)];
A2 = [chi(4) 0 0;
     0 chi(11) 0;
     0 0 chi(18)];
B2 = [chi(5) 0 0;
     0 chi(12) 0;
     0 0 chi(19)];
C2 = [chi(6) 0 0;
     0 chi(13) 0;
     0 0 chi(20)];
D2 = [chi(7) chi(8) chi(9);
     chi(14) chi(15) chi(16);
     chi(21) chi(22) chi(23)];
E2 = [chi(10) 0 0;
     0 chi(17) 0;
     0 0 chi(24)];
G1 = [0;0;m*g];
G_tras = [0;0;m*g];

end

