clear
clc

T_step = 200; %Time step
Z = 0.5*ones(1,T_step); %According to Figure 18.3 in the advanced text book, the water measured level is around 0.5m
measure_noise = 0.1*randn(1,T_step); %Measurement noise
Z = Z + measure_noise; %Add measurement noise

%Parameters for the continual state-space model, according to Example 18.2 in the advanced text book
A_tank=0.1;
Kp=0.002;
A_cont=[0,-1/A_tank;0,0];
B_cont=[Kp/A_tank;0];
C_cont=[1,0];
D_cont=[0];
Ts=0.1;

%Generates a state-space model
sys_cont=ss(A_cont,B_cont,C_cont,D_cont);

%Discretizie the continual state space model
sys_disc=c2d(sys_cont,Ts);
A=sys_disc.a;
C=sys_disc.c;

X=[0;0]; %Initial state of the system
P=[1,0;0,1]; %Covariance matrix of the estimation error
Q=[0.01,0;0,1e-4]; %Process noise covariance
R=[1e-6]; %Measurement noise covariance

for i = 1:T_step
    X_ = A*X; %Predicted state estimate
    P_ = A*P*A'+Q; %Error covariance
    K = P_*C'/(C*P_*C'+R); %Kalman Filter gain
    X = X_+K*(Z(i)-C*X_); %Update state estimation with measurement
    P = (eye(2)-K*C)*P_; %Covariance of predicted state estimate error, update
end

K