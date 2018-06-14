function f = fun_objective_mhe(x_est_matrix,...
    y_meas_mhe_array,u_mhe_array,model_params,covars,x_est_init_error,n,N_mhe,Ts)

%Duo: to estimate K, define parameter d
d = model_params.d;

T = model_params.T;

Q = covars.Q;
R = covars.R;

J_km1 = 0;

y1_meas_mhe_array = y_meas_mhe_array(1,:);

for k = 1:N_mhe
    u_k = u_mhe_array(1,k);
    x_k = x_est_matrix(:,k);
    x1_k = x_k(1);
    x2_k = x_k(2);
    
    %Duo: x3_k is K now
    x3_k = x_k(3);
    
    h1_k = x1_k;
    y1_meas_k = y1_meas_mhe_array(1,k);
    v1_k = y1_meas_k - h1_k;
    v_k = [v1_k];

    if k <= N_mhe-1
        x_kp1 = x_est_matrix(:,k+1);
        x1_kp1 = x_kp1(1);
        x2_kp1 = x_kp1(2);
        
        %Duo: x3_kp1 is K now
        x3_kp1 = x_kp1(3);
        
        dx1_dt_k = x2_k;
        
        %Duo: x3_k at the position of K, d is a constant now
        dx2_dt_k = (-x2_k + x3_k*u_k + d)/T;
        
        %Duo: dx3_dt_k is 0 now
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