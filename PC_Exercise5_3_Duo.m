clear
clc

%Loads data from file into workspace.
load airheater_logfile.txt;

L=length(airheater_logfile);%length of logfile
u=airheater_logfile(:,2); %Control signal to the heater, u [V]
Tout=airheater_logfile(:,3); %Outlet temperature, T_out [C]
dt=0.1; %Sampling interval

dTHeat_dt = zeros(L-1,1);
%Calculate dTHeat_dt using Euler forward
for i=1:L-1
    dTHeat_dt(i) = (Tout(i+1)-Tout(i))/dt;
end

%Contraints:
theta_d_min=1;theta_d_max=10;
Tenv_min=15;Tenv_max=250;
theta_t_min=0;theta_t_max=25;
Kh_min=0;Kh_max=10;
x_lb=[theta_d_min,Tenv_min,theta_t_min,Kh_min];
x_ub=[theta_d_max,Tenv_max,theta_t_max,Kh_max];

%--------------------------------------------------
%fmincon:
x_guess=[4,15,25,3];
Aineq=[]; Bineq=[]; Aeq=[]; Beq=[];

fun_objective_handle = @(x)fun_Ex5_objective(x,u,Tout,L,dTHeat_dt);%Local function
%fun_constraints_handle = @(x)fun_constraints(x,params_model);%Local function
optim_options=optimset('Display','on');
%optim_options=optimset('Algorithm','sqp','Display','on');
[x_opt,fval,exitflag,output,lambda,grad,hessian] =...
fmincon(fun_objective_handle,x_guess,Aineq,Bineq,Aeq,Beq,x_lb,x_ub,[],optim_options);

disp('-------------------')
disp('Optimal solution:')
f_min=fun_Ex5_objective(x_opt,u,Tout,L,dTHeat_dt)
theta_d_opt=x_opt(1)
Tenv_opt=x_opt(2)
theta_t_opt=x_opt(3)
Kh_opt=x_opt(4)