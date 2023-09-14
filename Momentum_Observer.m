function [Tu_est_next] = Momentum_Observer(chi_uav,u_ref,u,u_p,Tu_est,L,ts,Gain)


    C = function_C(chi_uav,u, L);

    u_tot = Model_um(u_p, u, chi_uav, L);
    p_p     = u_tot + C'*u;
    
    B = -C'*u;
     
    p_p_est = u_ref - B + Tu_est;
    
    Tu_est_p = Gain*eye(4)*(p_p - p_p_est);
    
    Tu_est_next = Tu_est + ts*Tu_est_p;
end