function x_next_RK4 = UAV_Dinamica_RK4(chi_uav,x,vref,L,ts)

    k1 = f_Dinamica_UAV(x, vref, L,chi_uav);   % new 
    k2 = f_Dinamica_UAV(x + ts/2*k1, vref, L,chi_uav); % new
    k3 = f_Dinamica_UAV(x + ts/2*k2, vref,  L,chi_uav); % new
    k4 = f_Dinamica_UAV(x + ts*k3, vref, L,chi_uav); % new
    x_next_RK4 = x +ts/6*(k1 +2*k2 +2*k3 +k4);
    
end