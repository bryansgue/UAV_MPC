function [E] = T_fuction(X)

E = zeros(6,6);

E(3,3) = X(15);

E(6,6) = X(16);

end

