function x_next_RK4 = UAV_Dinamica_RK4_T(chi_uav,x,vref,L,ts, Tu)

    k1 = f_Dinamica_UAV_T(x, vref, L,chi_uav, Tu);   % new 
    k2 = f_Dinamica_UAV_T(x + ts/2*k1, vref, L,chi_uav, Tu); % new
    k3 = f_Dinamica_UAV_T(x + ts/2*k2, vref,  L,chi_uav, Tu); % new
    k4 = f_Dinamica_UAV_T(x + ts*k3, vref, L,chi_uav, Tu); % new
    x_next_RK4 = x +ts/6*(k1 +2*k2 +2*k3 +k4);
    
end