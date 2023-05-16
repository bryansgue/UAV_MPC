function s_next = UAV_dynamic_Casadi(f,ts,s,F)

    k1 = full(f(s,F));
    k2 = full(f(s + ts/2*k1,F));
    k3 = full(f(s + ts/2*k2,F));
    k4 = full(f(s + ts*k3,F));

    s_next = s +ts/6*(k1 +2*k2 +2*k3 +k4); % new



end