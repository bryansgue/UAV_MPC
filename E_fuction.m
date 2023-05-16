function [E] = E_fuction(X)

E = zeros(6,6);
E(3,3) = X(12);
E(4,4) = X(13);
E(5,5) = X(14);
E(6,6) = X(15);

end

