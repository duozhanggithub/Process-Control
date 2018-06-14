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

%Initialization:
theta_d_min=1;theta_d_max=10;N_theta_d=10;
theta_d_array=linspace(theta_d_min,theta_d_max,N_theta_d);

Tenv_min=15;Tenv_max=25;N_Tenv=10;
Tenv_array=linspace(Tenv_min,Tenv_max,N_Tenv);

theta_t_min=15;theta_t_max=25;N_theta_t=10;
theta_t_array=linspace(theta_t_min,theta_t_max,N_theta_t);

Kh_min=1;Kh_max=10;N_Kh=10;
Kh_array=linspace(Kh_min,Kh_max,N_Kh);

f_min=inf;
theta_d_opt=-inf;
Tenv_opt=-inf;
theta_t_opt=-inf;
Kh_opt=-inf;

tic;

for k_theta_d=1:length(theta_d_array)
    theta_d=round(theta_d_array(k_theta_d));%use round to ensure the time delay is an integer         

    for k_Tenv=1:length(Tenv_array)
        Tenv=Tenv_array(k_Tenv);

        for k_theta_t=1:length(theta_t_array)
            theta_t=theta_t_array(k_theta_t);
            
            for k_Kh=1:length(Kh_array)
                Kh=Kh_array(k_Kh);
                
                %Objective function:
                u_delay = zeros(L-1,1);
                u_delay(theta_d:end) = u(1:L-theta_d);%implement time delay         
                Tout_est = Tenv + (Kh*u_delay - theta_t*dTHeat_dt);%calculate Tout
                f = sqrt(mean((Tout(1:L-1) - Tout_est).^2));%calculate root measn square error
                
                %Improving the previous solution:
                if f <= f_min,
                    f_min=f;
                    theta_d_opt=theta_d;
                    Tenv_opt=Tenv;
                    theta_t_opt=theta_t;
                    Kh_opt=Kh;
                end
                
            end
        end
    end
end

%Finding the optimal solution:

disp('-------------------')
disp('Optimal solution:')

f_min
theta_d_opt
Tenv_opt
theta_t_opt
Kh_opt