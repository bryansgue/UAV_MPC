function [opti, H0] = SolverUAV3D_MPC(h,v,hd,N,x_N,v_N,args,solver,k)

    s = 12;
    u = 4;
    args.p(1:s) = [h]; % Generacion del estado del sistema
    
    for i = 1:N % z
        args.p(s*i+1:s*i+s)=hd(:,k+i);
    end 
    
    args.x0 = [reshape(x_N',s*(N+1),1);reshape(v_N',size(v_N,2)*N,1)]; % initial value of the optimization variables
    
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        
    opti = reshape(full(sol.x(s*(N+1)+1:end))',u,N)';
    H0 = reshape(full(sol.x(1:s*(N+1)))',s,N+1)';
end

