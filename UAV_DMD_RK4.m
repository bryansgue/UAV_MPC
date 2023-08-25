function x_next_RK4 = UAV_DMD_RK4(A,B,x,vref,L,ts)

    k1 = f_DMD(x, vref, L,A,B);   % new 
    k2 = f_DMD(x + ts/2*k1, vref, L,A,B); % new
    k3 = f_DMD(x + ts/2*k2, vref,  L,A,B); % new
    k4 = f_DMD(x + ts*k3, vref, L,A,B); % new
    x_next_RK4 = x +ts/6*(k1 +2*k2 +2*k3 +k4);
    
end