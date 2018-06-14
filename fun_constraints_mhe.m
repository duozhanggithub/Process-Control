function [cineq,ceq]=fun_constraints_mhe(x_est_matrix,...
    y_meas_mhe_array,u_mhe_array,model_params,covars,x_est_init_error,n,N,Ts)
cineq = [];   % Compute nonlinear inequalities. Calculated below.
ceq = [];   % Compute nonlinear equalities.
end