function [opti, H0] = SolverUAV_MPC(h,hd,N,x_N,v_N,args,solver,k)

    args.p(1:4) = h(:,k); % Generacion del estado del sistema
    
    
    for i = 1:N % z
        args.p(4*i+1:4*i+4)=[hd(1,k+i);hd(2,k+i);hd(3,k+i);hd(4,k+i)];
    end 
    
    args.x0 = [reshape(x_N',4*(N+1),1);reshape(v_N',4*N,1)]; % initial value of the optimization variables

    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    toc
 
    opti = reshape(full(sol.x(4*(N+1)+1:end))',4,N)';
    H0 = reshape(full(sol.x(1:4*(N+1)))',4,N+1)';



end

