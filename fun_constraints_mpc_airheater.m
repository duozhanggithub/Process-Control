function [cineq,ceq]=fun_constraints_mpc_airheater(u,u_opt_km1,state_est,Temp_env_k,...
    Temp_sp_to_mpc_array,model_params,mpc_costs,N_pred,Ts)
cineq =  [];   % Compute nonlinear inequalities.
ceq = [];   % Compute nonlinear equalities.
end