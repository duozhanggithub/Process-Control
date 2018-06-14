%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Finn Aakre Haugen
%15 04 2018
%MHE with model with 2 state variables and 1 disturbance as augmented state:
%dx1_dt = x2 + w1
%dx2/dt = (-x2 + K*u + d)/T + w2
%y = x1 + v
%where K = x3 is to be estimated.
%
%Note 1: Using matrix as optim variable.
%Note 2: Objective and constraints functions for fmincon are defined
%as local functions at the end of this script.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('-------------------------------');
disp('Moving Horizon Estimator for:')
disp('dx1_dt=x2; dx2/dt=(-x2+K*u+d)/T;');
disp('x1, x2, x3 = K are estimated.');
disp('y=x1 is measured.');
disp('');
%-------------------------------------------
close all
clear all
format compact
commandwindow
%-------------------------------------------
%Model params:

%Duo: because we will estimate K, so d is a constant now
d = 1; %Gain
T = 2;%s Time constant
n = 3;%Number of state variables

%Duo: because we will estimate K, so d is a constant now
model_params.d = d;
model_params.T = T;%model_params is struct. K is field.
cov_process_disturb_w1 = .001;
cov_process_disturb_w2 = .001;
cov_process_disturb_w3 = .001;
cov_process_disturb_w = ...
    diag([cov_process_disturb_w1,cov_process_disturb_w2,...
    cov_process_disturb_w3]);
cov_meas_noise_v1 = .01;
cov_meas_noise_v = diag([cov_meas_noise_v1]);
%--------------------------
%Time settings:
Ts = 0.5;%s
t_start = 0;%s
t_stop = 20;%s
t_array = [t_start:Ts:(t_stop-Ts)];%Array for storage
N = length(t_array);
t_mhe = 5
N_mhe = floor(t_mhe/Ts)
number_optim_vars = n*N_mhe

%-----------------------------------
%Preallocation of arrays for storage:

u_sim_array = t_array*0;
x1_sim_array = t_array*0;
x2_sim_array = t_array*0;
x3_sim_array = t_array*0;
y1_sim_array = t_array*0;
x1_est_optim_plot_array = t_array*0;
x2_est_optim_plot_array = t_array*0;
x3_est_optim_plot_array = t_array*0;

%-----------------------------------
%Sim initialization:

x1_sim_init = 2;
x2_sim_init = 3;
x1_sim_k = x1_sim_init;
x2_sim_k = x2_sim_init;

%-----------------------------------
%MHE initialization:

mhe_array = zeros(1,N_mhe);

x1_est_init_guess = 0;
x2_est_init_guess = 0;
x3_est_init_guess = 2;
x1_est_optim_array = zeros(1,N_mhe) + x1_est_init_guess;
x2_est_optim_array = zeros(1,N_mhe) + x2_est_init_guess;
x3_est_optim_array = zeros(1,N_mhe) + x3_est_init_guess;

x_est_guess_matrix = ...
    [x1_est_optim_array;x2_est_optim_array;x3_est_optim_array];

u_mhe_array = mhe_array*0;
y1_meas_mhe_array = mhe_array*0;

%-----------------------------------
%Figure size etc.:

fig_posleft=8;fig_posbottom=2;fig_width=24;fig_height=18;
fig_pos_size_1=[fig_posleft,fig_posbottom,fig_width,fig_height];

h = figure(1);
set(gcf,'Units','centimeters','Position',fig_pos_size_1);
figtext='Moving Horizon Estimator';
set(gcf,'Name',figtext,'NumberTitle','on')
%-----------------------------------
%Sim loop:

for k = 1:N
    t_k = k*Ts;
    %-----------------------------
    %Process simulator:
    
    %Duo: we are estimating K. so change d_k to K
    if t_k < 2 
        u_k = 2;
        K_k = 1;
    end
    if t_k >= 2 %Change of u
        u_k = 2;
    end
    if t_k >= 8 %Change of K
        K_k = 1;
    end
    if t_k >= 10 %Change of u
        u_k = 4;
    end
    if t_k >= 15 %Change of K
        K_k = 2;
    end
    
    %Derivatives:
    dx1_sim_dt_k = x2_sim_k;
    
    %Duo: now we are estimating K, d is a constant
    dx2_sim_dt_k = (-x2_sim_k + K_k*u_k + d)/T;
    f1_sim_k = x1_sim_k + Ts*dx1_sim_dt_k;
    f2_sim_k = x2_sim_k + Ts*dx2_sim_dt_k;
    
    %Integration and adding disturbance:
    w1_sim_k = sqrt(cov_process_disturb_w1)*randn;
    w2_sim_k = sqrt(cov_process_disturb_w2)*randn;
    x1_sim_kp1 = f1_sim_k + w1_sim_k;
    x2_sim_kp1 = f2_sim_k + w2_sim_k;
    
    %Calculating output and adding meas noise:
    v1_sim_k = sqrt(cov_meas_noise_v1)*randn;
    y1_sim_k = x1_sim_k + v1_sim_k;
    
    %Storage:
    t_array(k) = t_k;
    u_sim_array(k) = u_k;
    x1_sim_array(k) = x1_sim_k;
    x2_sim_array(k) = x2_sim_k;
    
    %Duo: x3 is K, not d_k now
    x3_sim_array(k) = K_k;
    
    y1_sim_array(k) = y1_sim_k;    
    
    %Preparing for time shift:
    x1_sim_k = x1_sim_kp1;
    x2_sim_k = x2_sim_kp1;

    %Updating u and y for use in MHE:
    u_mhe_array = [u_mhe_array(2:N_mhe),u_k];
    y1_meas_mhe_array = [y1_meas_mhe_array(2:N_mhe),y1_sim_k];
    y_meas_mhe_array = [y1_meas_mhe_array];
    %--------------------------------------------------------------------
    if k > N_mhe

    Q = cov_process_disturb_w;
    R = cov_meas_noise_v;
    covars.Q = Q;
    covars.R = R;

    %Matrices defining linear constraints for use in fmincon:
    A_ineq = []; B_ineq = []; A_eq = []; B_eq = [];

    %fmincon initialization:
    x1_est_init_error = 0;
    x2_est_init_error = 0;
    x3_est_init_error = 0;
    x_est_init_error=[x1_est_init_error;x2_est_init_error;x3_est_init_error];

    %Guessed optim states:
    %Guessed present state (x_k) is needed to calculate optimal present meas
    %(y_k). Model is used in prediction:
    x1_km1 = x1_est_optim_array(N_mhe);
    x2_km1 = x2_est_optim_array(N_mhe);
    x3_km1 = x3_est_optim_array(N_mhe);

    dx1_dt_km1 = x2_km1;
    
    %Duo: the x3_km1 at the position of K
    dx2_dt_km1 = (-x2_km1 + x3_km1*u_k + d)/T;
    
    %Duo: the derivative of x3_km1 is 0 
    dx3_dt_km1 = 0;
    
    x1_pred_k = x1_km1 + Ts*dx1_dt_km1;
    x2_pred_k = x2_km1 + Ts*dx2_dt_km1;
    x3_pred_k = x3_km1 + Ts*dx3_dt_km1;
    
    %Now, guessed optimal states are:
    x1_est_guess_array = ...
        [x1_est_optim_array(2:N_mhe),x1_pred_k];
    x2_est_guess_array = ...
        [x2_est_optim_array(2:N_mhe),x2_pred_k];
    x3_est_guess_array = ...
        [x3_est_optim_array(2:N_mhe),x3_pred_k];
    x_est_guess_matrix = ...
        [x1_est_guess_array;x2_est_guess_array;x3_est_guess_array];

    %Lower and upper limits of optim variables:
    x1_est_max = 100;
    x2_est_max = 10;
    x3_est_max = 10;
    x1_est_max_array = zeros(1,N_mhe) + x1_est_max;
    x2_est_max_array = zeros(1,N_mhe) + x2_est_max;
    x3_est_max_array = zeros(1,N_mhe) + x3_est_max;

    x1_est_min = -100;
    x2_est_min = -10;
    x3_est_min = -10;
    x1_est_min_array = zeros(1,N_mhe) + x1_est_min;
    x2_est_min_array = zeros(1,N_mhe) + x2_est_min;
    x3_est_min_array = zeros(1,N_mhe) + x3_est_min;

    x_est_ub_matrix = [x1_est_max_array;x2_est_max_array;x3_est_max_array];
    x_est_lb_matrix = [x1_est_min_array;x2_est_min_array;x3_est_min_array];

    %Creating function handles:
    fun_objective_handle = ...
        @(x_est_matrix) fun_objective_mhe(x_est_matrix,...
        y_meas_mhe_array,u_mhe_array,model_params,covars,x_est_init_error,n,N_mhe,Ts);
    fun_constraints_handle = ...
        @(x_est_matrix) fun_constraints_mhe(x_est_matrix,...
        y_meas_mhe_array,u_mhe_array,model_params,covars,x_est_init_error,n,N_mhe,Ts);

    %Calculating MHE estimate using fmincon:
    %fmincon_options = optimoptions(@fmincon);
    fmincon_options = optimoptions(@fmincon,'display','none');
%     fmincon_options = optimoptions(@fmincon,'algorithm','sqp','display','none');

    [x_est_optim_matrix,fval,exitflag,output,lambda,grad,hessian] = ...
    fmincon(fun_objective_handle,x_est_guess_matrix,A_ineq,...
        B_ineq,A_eq,B_eq,x_est_lb_matrix,x_est_ub_matrix,...
        fun_constraints_handle,fmincon_options);

    x1_est_optim_array = x_est_optim_matrix(1,:);
    x2_est_optim_array = x_est_optim_matrix(2,:);
    x3_est_optim_array = x_est_optim_matrix(3,:);

    % fval

    end %if
    x1_est_optim_plot_array(k) = x1_est_optim_array(end);
    x2_est_optim_plot_array(k) = x2_est_optim_array(end);
    x3_est_optim_plot_array(k) = x3_est_optim_array(end);
    
    
    %Continuous plotting:
    x_lim_array=[t_start,t_stop];
    if (k>1 & k<N)
        if k < N_mhe
            pause(1);
        else
            pause(0);
        end

        subplot(4,1,1)
        plot([t_array(k-1),t_array(k)],...
            [u_sim_array(k-1),u_sim_array(k)],'b-o');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
            ylim([0 10]);
            title('u')
            %ylabel('[m]')
            %xlabel('t [s]')
        end

        subplot(4,1,2)
        plot([t_array(k-1),t_array(k)],...
            [x1_est_optim_plot_array(k-1),x1_est_optim_plot_array(k)],'r-o',...
            [t_array(k-1),t_array(k)],...
            [x1_sim_array(k-1),x1_sim_array(k)],'b-o');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
            ylim([0 100]);
            title('x1\_sim = blue. x1\_mhe\_est = red.')
            %ylabel('[m]')
            %xlabel('t [s]')
        end

        subplot(4,1,3)
        plot([t_array(k-1),t_array(k)],...
            [x2_est_optim_plot_array(k-1),x2_est_optim_plot_array(k)],'r-o',...
            [t_array(k-1),t_array(k)],...
            [x2_sim_array(k-1),x2_sim_array(k)],'b-o');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
            ylim([0 15]);
            title('x2\_sim = blue. x2\_mhe\_est = red.')
            %ylabel('[m]')
            %xlabel('t [s]')
        end

        subplot(4,1,4)
        plot([t_array(k-1),t_array(k)],...
            [x3_est_optim_plot_array(k-1),x3_est_optim_plot_array(k)],'r-o',...
            [t_array(k-1),t_array(k)],...
            [x3_sim_array(k-1),x3_sim_array(k)],'b-o');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
            ylim([0 10]);
            title('x3\_sim = K\_sim = blue. x3\_mhe\_est = d\_mhe\_est = red.')
            %ylabel('[m]')
            xlabel('t [s]')
        end
    end %if (k>1 & k<N)
end %sim loop

%----------------------------------------------------
%Printing figure as pdf file:

%saveas(h,'example_mhe','pdf')