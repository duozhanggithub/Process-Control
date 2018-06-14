%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Finn Aakre Haugen
%08 04 2018
%MHE with model with 2 state variables and 1 disturbance as augmented state:
%dx1_dt = x2 + w1
%dx2/dt = (-x2 + K*u + d)/T + w2
%y = x1 + v
%where d = x3 is to be estimated.
%
%Note 1: Using matrix as optim variable.
%Note 2: Objective and constraints functions for fmincon are defined
%as local functions at the end of this script.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('-------------------------------');
disp('Moving Horizon Estimator for:')
disp('dx1_dt=x2; dx2/dt=(-x2+K*u)/T;');
disp('x1, x2, x3 = d are estimated.');
disp('y=x1 is measured.');
disp('');
%-------------------------------------------
close all
clear all
format compact
commandwindow
%-------------------------------------------
%Model params:
K = 1; %Gain
T = 2;%s Time constant
n = 3;%Number of state variables
% model_params.K = K;%p is struct. K is field. (Convenient way to collect parms.)
model_params.T = T;
model_params.K = K;
cov_process_disturb_w1 = .01;
cov_process_disturb_w2 = .01;
cov_process_disturb_w3 = .01;
cov_process_disturb_w = ...
    diag([cov_process_disturb_w1,cov_process_disturb_w2,cov_process_disturb_w3]);
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
fig_posleft=8;fig_posbottom=2;fig_width=26;fig_height=18;
fig_pos_size_1=[fig_posleft,fig_posbottom,fig_width,fig_height];

figure(1),
set(gcf,'Units','centimeters','Position',fig_pos_size_1);
figtext='Moving Horizon Estimator';
set(gcf,'Name',figtext,'NumberTitle','on')
%-----------------------------------
%Sim loop:
for k = 1:N
    t_k = k*Ts;
    %-----------------------------
    %Process simulator:
    if t_k < 2 
        u_k = 2;
        d_k = 1;
    end
    if t_k >= 2 %Change of u
        u_k = 2;
    end
    if t_k >= 5 %Change of u
        u_k = 2;
    end
    if t_k >= 8 %Change of K
        d_k = 1;
    end
    if t_k >= 10 %Change of u
        u_k = 6;
    end
    if t_k >= 15 %Change of u
        u_k = 2;
    end
    if t_k >= 16 %Change of K
        d_k = 1;
    end
    %Derivatives:
    dx1_sim_dt_k = x2_sim_k;
    dx2_sim_dt_k = (-x2_sim_k + K*u_k + d_k)/T;
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
    x3_sim_array(k) = d_k;
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

    cov_init_state = diag([0.01,0.01,0.01]);
    P = cov_init_state;
    Q = cov_process_disturb_w;
    R = cov_meas_noise_v;
    covars.P = P;
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
    dx2_dt_km1 = (-x2_km1 + K*u_k + x3_km1)/T;
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


%     %Guessed optim values:
%     x1_est_guess_array = ...
%         [x1_est_optim_array(2:N_mhe),x1_est_optim_array(N_mhe)];
%     x2_est_guess_array = ...
%         [x2_est_optim_array(2:N_mhe),x2_est_optim_array(N_mhe)];
%     x3_est_guess_array = ...
%         [x3_est_optim_array(2:N_mhe),x3_est_optim_array(N_mhe)];
%     x_est_guess_matrix = ...
%         [x1_est_guess_array;x2_est_guess_array;x3_est_guess_array];

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
        plot([t_array(k-1),t_array(k)],[u_sim_array(k-1),u_sim_array(k)],'b-o');
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
        plot([t_array(k-1),t_array(k)],[x1_est_optim_plot_array(k-1),x1_est_optim_plot_array(k)],'r-o',...
            [t_array(k-1),t_array(k)],[x1_sim_array(k-1),x1_sim_array(k)],'b-o');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
            ylim([0 50]);
            title('x1\_sim = blue. x1\_mhe\_est = red.')
            %ylabel('[m]')
            %xlabel('t [s]')
        end

        subplot(4,1,3)
        plot([t_array(k-1),t_array(k)],[x2_est_optim_plot_array(k-1),x2_est_optim_plot_array(k)],'r-o',...
            [t_array(k-1),t_array(k)],[x2_sim_array(k-1),x2_sim_array(k)],'b-o');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
            ylim([0 10]);
            title('x2\_sim = blue. x2\_mhe\_est = red.')
            %ylabel('[m]')
            %xlabel('t [s]')
        end

        subplot(4,1,4)
        plot([t_array(k-1),t_array(k)],[x3_est_optim_plot_array(k-1),x3_est_optim_plot_array(k)],'r-o',...
            [t_array(k-1),t_array(k)],[x3_sim_array(k-1),x3_sim_array(k)],'b-o');
        if k==2
            hold on
            grid minor
            xlim(x_lim_array);
            ylim([0 3]);
            title('x3\_sim = K\_sim = blue. x3\_mhe\_est = d\_mhe\_est = red.')
            %ylabel('[m]')
            xlabel('t [s]')
        end
    end %if (k>1 & k<N)
end %sim loop

% %------------------------
% %Batch plotting:
% fig_posleft=8;fig_posbottom=1;fig_width=26;fig_height=18;
% fig_pos_size_1=[fig_posleft,fig_posbottom,fig_width,fig_height];
% figure(1),
% set(gcf,'Units','centimeters','Position',fig_pos_size_1);
% figtext='MHE';
% set(gcf,'Name',figtext,'NumberTitle','on')
% x_label_string=('t [s]');
% 
% subplot(4,1,1)
% plot(t_array,u_sim_array,'b-o')
% %xlim(x_lim_array);
% ylim([-1 11]);
% title('u')
% % ylabel('[m]')
% xlabel(x_label_string)
% grid minor
% 
% subplot(4,1,2)
% plot(t_array,x1_est_optim_plot_array,'r-o',...
%     t_array,x1_sim_array,'b-o')
% % xlim(x_lim_array);
% % ylim([0,2]);
% title('x1\_sim = blue. x1\_mhe\_est = red.')
% %ylabel('')
% xlabel(x_label_string)
% grid minor
% 
% subplot(4,1,3)
% plot(t_array,x2_est_optim_plot_array,'r-o',...
%     t_array,x2_sim_array,'b-o')
% % xlim(x_lim_array);
% % ylim([0,2]);
% title('x2\_sim = blue. x2\_mhe\_est = red.')
% %ylabel('')
% xlabel(x_label_string)
% grid minor
% 
% subplot(4,1,4)
% plot(t_array,x3_est_optim_plot_array,'r-o',...
%     t_array,x3_sim_array,'b-o')
% % xlim(x_lim_array);
% % ylim([0,2]);
% title('x3\_sim = K\_sim = blue. x3\_mhe\_est = K\_mhe\_est = red.')
% %ylabel('')
% xlabel(x_label_string)
% grid minor

% % elapsed_time=toc
% % t_iteration=elapsed_time/N_sim
% % sim_speed=Ts/t_iteration

%----------------------------------------------------
%Defining local functions:
%----------------------------------------------------

function f = fun_objective_mhe(x_est_matrix,...
    y_meas_mhe_array,u_mhe_array,model_params,covars,x_est_init_error,n,N_mhe,Ts)

%K = model_params.K;
T = model_params.T;
K = model_params.K;

P = covars.P;
Q = covars.Q;
R = covars.R;

J_km1 = x_est_init_error'*inv(P)*x_est_init_error;

y1_meas_mhe_array = y_meas_mhe_array(1,:);

for k = 1:N_mhe
    u_k = u_mhe_array(1,k);
    x_k = x_est_matrix(:,k);
    x1_k = x_k(1);
    x2_k = x_k(2);
    x3_k = x_k(3);
    h1_k = x1_k;
    y1_meas_k = y1_meas_mhe_array(1,k);
    v1_k = y1_meas_k - h1_k;
    v_k = [v1_k];

    if k <= N_mhe-1
        x_kp1 = x_est_matrix(:,k+1);
        x1_kp1 = x_kp1(1);
        x2_kp1 = x_kp1(2);
        x3_kp1 = x_kp1(3);
        
        dx1_dt_k = x2_k;
        dx2_dt_k = (-x2_k + K*u_k + x3_k)/T;
        dx3_dt_k = 0;
        f1_k = x1_k + Ts*dx1_dt_k;
        f2_k = x2_k + Ts*dx2_dt_k;
        f3_k = x3_k + Ts*dx3_dt_k;
        w1_k = x1_kp1 - f1_k;
        w2_k = x2_kp1 - f2_k;
        w3_k = x3_kp1 - f3_k;
        w_k = [w1_k,w2_k,w3_k]';
    end
        
    dJ_k = w_k'*inv(Q)*w_k + v_k'*inv(R)*v_k;
    J_k = J_km1 + dJ_k;

    %Time shift:
    J_km1 = J_k;
end %for k=1:N_mhe
f = J_k;
end

%------------------------------------------------
function [cineq,ceq]=fun_constraints_mhe(x_est_matrix,...
    y_meas_mhe_array,u_mhe_array,model_params,covars,x_est_init_error,n,N,Ts)
cineq = [];   % Compute nonlinear inequalities. Calculated below.
ceq = [];   % Compute nonlinear equalities.
end
%------------------------------------------------
