%Finn Haugen, USN
%16 April 2018
%MPC control of simulated air heater
%http://home.usn.no/finnh/air_heater/
%----------------------------------

clear all
close all
format short

%----------------------------------
%Process params:

gain = 3.5; %[deg C]/[V]
theta_const = 23; %[s]
theta_delay = 3; %[s]
model_params.gain = gain;
model_params.theta_const = theta_const;
model_params.theta_delay = theta_delay;

Temp_env_k = 25; %[deg C]

%----------------------------------
%Time settings:

Ts = 0.5; %Time-step [s]
t_pred_horizon = 8;

N_pred = t_pred_horizon/Ts;
t_start = 0;
t_stop = 300;
N_sim = (t_stop-t_start)/Ts;
t = [t_start:Ts:t_stop-Ts];

%-----------------------
%MPC costs:

C_e = 1;
C_du = 20;
mpc_costs.C_e = C_e;
mpc_costs.C_du = C_du;

%----------------------------------
%Defining sequence for temp_out setpoint:

Temp_sp_const = 30; %[C]   
Ampl_step = 2; %[C]
Slope = -0.04; %[C/s]
Ampl_sine = 1; %[C]
T_period = 50; %[s]

t_const_start = t_start;t_const_stop = 100;
t_step_start = t_const_stop;t_step_stop = 150;
t_ramp_start = t_step_stop;t_ramp_stop = 200;
t_sine_start = t_ramp_stop;t_sine_stop = 250;
t_const2_start = t_sine_stop;t_const2_stop = t_stop;

for k = 1:N_sim
    if (t(k) >= t_const_start & t(k) < t_const_stop),
        Temp_sp_array(k) = Temp_sp_const;
    end
    if (t(k) >= t_step_start & t(k) < t_step_stop),
        Temp_sp_array(k)=Temp_sp_const + Ampl_step;
    end
    if (t(k) >= t_ramp_start & t(k) < t_ramp_stop),
        Temp_sp_array(k) = Temp_sp_const+Ampl_step+Slope*(t(k)-t_ramp_start);
    end
    if (t(k) >= t_sine_start & t(k) < t_sine_stop),
        Temp_sp_array(k) = ...
            Temp_sp_const+Ampl_sine*sin(2*pi*(1/T_period)*(t(k)-t_sine_start));
    end
    if (t(k) >= t_const2_start),
        Temp_sp_array(k) = Temp_sp_const;
    end
end

%----------------------------------
%Initialization:

u_init = 0;
N_delay = floor(theta_delay/Ts) + 1;
delay_array = zeros(1,N_delay) + u_init;

%----------------------------------
%Initial guessed optimal control sequence:

Temp_heat_sim_k = 0; %[C]
Temp_out_sim_k = 28; %[C]
d_sim_k = -0.5;

%----------------------------------
%Initial values for estimator:

Temp_heat_est_k = 0; %[C]
Temp_out_est_k = 25; %[C]
d_est_k = 0; %[V]

%----------------------------------
%Initial guessed optimal control sequence:
u_guess = zeros(N_pred,1) + u_init;

%----------------------------------
%Initial value of previous optimal value:
u_opt_km1 = u_init;

%----------------------------------
%Defining arrays for plotting:

t_plot_array = zeros(1,N_sim);
Temp_out_sp_plot_array = zeros(1,N_sim);
Temp_out_sim_plot_array = zeros(1,N_sim);
u_plot_array = zeros(1,N_sim);
d_est_plot_array = zeros(1,N_sim);
d_sim_plot_array = zeros(1,N_sim);

%----------------------------------
%Matrices defining linear constraints for use in fmincon:

A = [];
B = [];
Aeq = [];
Beq = [];

%----------------------------------
%Lower and upper limits of optim variable for use in fmincon:

u_max = 5;
u_min = 0;
u_ub = zeros(1,N_pred) + u_max;
u_lb = zeros(1,N_pred) + u_min;

u_delayed_k = 2;

%------------------------
%Calculation of observer gain for estimation of input disturbance, d:

A_est = [-1/theta_const, gain/theta_const; 0,0];
C_est = [1 0];
n_est = length(A_est);
Tr_est = 5;
T_est = Tr_est/n_est;
estimpoly = [T_est*T_est,sqrt(2)*T_est,1];
eig_est = roots(estimpoly);
K_est1 = acker(A_est',C_est',eig_est);
K_est = K_est1';

%-----------------------------------
%Figure settings:

fig_posleft=8;fig_posbottom=2;fig_width=24;fig_height=18;
fig_pos_size_1=[fig_posleft,fig_posbottom,fig_width,fig_height];

h = figure(1);
set(gcf,'Units','centimeters','Position',fig_pos_size_1);
figtext='MPC control of air heater';
set(gcf,'Name',figtext,'NumberTitle','on')

%----------------------------------
%For-loop for MPC of simulated process incl Kalman Filter:

tic

for k = 1:(N_sim-N_pred)
    t_k = t(k);
    t_plot_array(k)= t_k;
    
    %-----------------------
    %Observer for estimating input-disturbance d using Temp_out:
    %Note: The time-delayed u is used as control signal shere.
    e_est_k = Temp_out_sim_k - Temp_out_est_k;

    dTemp_heat_est_dt_k = ...
        (1/theta_const)*(-Temp_heat_est_k + gain*(u_delayed_k + d_est_k))...
        + K_est(1)*e_est_k;
    dd_est_dt_k = 0 + K_est(2)*e_est_k;

    Temp_heat_est_kp1 = Temp_heat_est_k + Ts*dTemp_heat_est_dt_k;
    d_est_kp1 = d_est_k + Ts*dd_est_dt_k;

    Temp_out_est_k = Temp_heat_est_k + Temp_env_k;

    %-----------------------
    %Storage for plotting:
    
    Temp_out_est_plot_array(k) = Temp_out_est_k;
    d_est_plot_array(k) = d_est_k;
        
    %-----------------------
    %Setpoint array to optimizer:
    
    Temp_sp_to_mpc_array = Temp_sp_array(k:k+N_pred);
    Temp_out_sp_plot_array(k) = Temp_sp_array(k);
        
    %-----------------------
    %Estimated state to optimizer:

    state_est.Temp_heat_est_k = Temp_heat_est_k;
    state_est.d_est_k = d_est_k;
   
    %-----------------------
    %Calculating optimal control sequence:

    fun_handle = @(u) fun_objectfunction_mpc_airheater...
        (u,u_opt_km1,state_est,Temp_env_k,...
    Temp_sp_to_mpc_array,model_params,mpc_costs,N_pred,Ts);

     fmincon_options = optimoptions(@fmincon,'display','none');
%     fmincon_options = optimoptions(@fmincon,'algorithm','active-set','display','none');
%     fmincon_options = optimoptions(@fmincon,'algorithm','sqp','display','none');
        [u_opt,fval,exitflag,output,lambda,grad,hessian] =...
    fmincon(fun_handle,u_guess,A,B,Aeq,Beq,u_lb,u_ub,@fun_constraints_mpc_airheater,fmincon_options);

    u_guess = u_opt; %Optimal solution to be used as guessed solution in next iteration.
    u_k = u_opt(1); %Optimal control signal (sample) to be applied
    u_plot_array(k) = u_k; %Storage for plotting
    u_opt_km1 = u_opt(1);

    %------------------------------
    %Applying optimal control signal to simulated process:    

    d_sim_k = -0.5;
    d_sim_plot_array(k) = d_sim_k;

    u_delayed_k = delay_array(N_delay);
    u_nondelayed_k = u_k;
    delay_array = [u_nondelayed_k,delay_array(1:end-1)];

    dTemp_heat_sim_dt_k = ...
        (1/theta_const)*(-Temp_heat_sim_k + gain*(u_delayed_k + d_sim_k));
    Temp_heat_sim_kp1 = Temp_heat_sim_k + Ts*dTemp_heat_sim_dt_k;
    Temp_out_sim_k = Temp_heat_sim_k + Temp_env_k;

    Temp_out_sim_plot_array(k) = Temp_out_sim_k;%Storage for plotting

    %------------------------------
    %Time shift for estimator and for simulator:
    
    Temp_heat_est_k = Temp_heat_est_kp1;
    d_est_k = d_est_kp1;
    
    Temp_heat_sim_k = Temp_heat_sim_kp1;
    
    %------------------------------
    %Continuous plotting:
    
    x_lim_array = [t_start,t_stop];
    if (k>1 & k<N_sim)
        
        pause(0.0);

        subplot(3,1,1)
        plot([t_plot_array(k-1),t_plot_array(k)],...
            [Temp_out_sp_plot_array(k-1),Temp_out_sp_plot_array(k)],'r-',...
            [t_plot_array(k-1),t_plot_array(k)],...
            [Temp_out_sim_plot_array(k-1),Temp_out_sim_plot_array(k)],'b-',...
            [t_plot_array(k-1),t_plot_array(k)],...
            [Temp_out_est_plot_array(k-1),Temp_out_est_plot_array(k)],'m-');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
            ylim([28 33]);
            title('Temp\_out\_sp = red. Temp\_out\_sim = blue. Temp\_out\_est = magenta.')
            ylabel('[deg C]')
            xlabel('t [s]')
        end

        subplot(3,1,2)
        plot([t_plot_array(k-1),t_plot_array(k)],...
            [u_plot_array(k-1),u_plot_array(k)],'b-');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
            ylim([0 5]);
            title('Control signal u')
            ylabel('[V]')
            xlabel('t [s]')
        end

        subplot(3,1,3)
        plot([t_plot_array(k-1),t_plot_array(k)],...
            [d_est_plot_array(k-1),d_est_plot_array(k)],'r-',...
            [t_plot_array(k-1),t_plot_array(k)],...
            [d_sim_plot_array(k-1),d_sim_plot_array(k)],'b-');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
%             ylim([0 10]);
            title('d\_est = red. d\_sim = blue.')
            %ylabel('[m]')
            xlabel('t [s]')
        end

    end %if (k>1 & k<N)

end

%Elapsed total time and avg loop time:
toc_total=toc
toc_loop=toc_total/k

%----------------------------------------------------
%Printing figure as pdf file:

% saveas(h,'example_mpc_air_heater','pdf')

%----------------------------------------------------

%Functions defined as local functions:

function f = fun_objectfunction_mpc_airheater(u,u_opt_km1,state_est,Temp_env_k,...
    Temp_sp_to_mpc_array,model_params,mpc_costs,N_pred,Ts)

gain = model_params.gain;
theta_const = model_params.theta_const;
theta_delay = model_params.theta_delay;

C_e = mpc_costs.C_e;
C_du = mpc_costs.C_du;

Temp_heat_k = state_est.Temp_heat_est_k;
d_k = state_est.d_est_k;

N_delay = floor(theta_delay/Ts) + 1;
delay_array = zeros(1,N_delay) + u(1);

% ru_km1 = u_opt_km1;
u_km1 = u(1);
J_km1 = 0;

%Applying optimal control signal to simulated process using explicit Euler:

for k = 1:N_pred
    
    u_k = u(k);
    Temp_sp_k = Temp_sp_to_mpc_array(k);
    
    %Time delay:
    u_delayed_k = delay_array(N_delay);
    u_nondelayed_k = u_k;
    delay_array = [u_nondelayed_k,delay_array(1:end-1)];

    %Solving diff eq:
    dTemp_heat_dt_k = ...
        (1/theta_const)*(-Temp_heat_k + gain*(u_delayed_k + d_k));
    Temp_heat_kp1 = Temp_heat_k + Ts*dTemp_heat_dt_k;
    Temp_out_k = Temp_heat_k + Temp_env_k;

    %Updating objective function:
    e_k = Temp_sp_k - Temp_out_k;
    du_k = (u_k - u_km1)/Ts;
    J_k = J_km1 + Ts*(C_e*e_k^2 + C_du*du_k^2);

    %Time shift:
    Temp_heat_k = Temp_heat_kp1;
    u_km1 = u_k;
    J_km1 = J_k;
end

f = J_k;

end

%--------------------------------

function [cineq,ceq]=fun_constraints_mpc_airheater(u,u_opt_km1,state_est,Temp_env_k,...
    Temp_sp_to_mpc_array,model_params,mpc_costs,N_pred,Ts)
cineq =  [];   % Compute nonlinear inequalities.
ceq = [];   % Compute nonlinear equalities.
end
