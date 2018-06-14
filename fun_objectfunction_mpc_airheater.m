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