function f=fun_Ex5_objective(x,u,Tout,L,dTHeat_dt)
thera_d=round(x(1));
Tenv=x(2);
thera_t=x(3);
Kh=x(4);

u_delay = zeros(L-1,1);
u_delay(thera_d:end) = u(1:L-thera_d);         
Tout_est = Tenv + (Kh*u_delay - thera_t*dTHeat_dt);
f = sqrt(mean((Tout(1:L-1) - Tout_est).^2));
end