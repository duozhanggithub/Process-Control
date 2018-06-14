%Finn Aakre Haugen (finn@techteach.no)
%210218
%----------------------------------------------------------
disp('-------------------')
disp('Newtons method to solve an')
disp('optimization problem:')
disp('min_x f(x)')
disp('s.t.')
disp('model: f=100*(x(2) - x(1)^2)^2 + (1 - x(1))^2')
%disp('params: p1 = 1, p2 = 2, p3 = 3')
%----------------------------------------------------------
format compact
clear all
close all

%p1=1;
%p2=2;
%p3=3;
x1_k=-1.9;
x2_k=2;
x_k=[x1_k;x2_k];
%h=0.01;
df=1;
k=0;
 while 1
    disp('-------------------')
    k=k+1
    %Gradient:
    G_k=[2*400*x1_k^3-400*x2_k*x1_k+2*x1_k-2;
         200*(x2_k - x1_k^2)];
    %Hessian:
    H_k=[1200*x1_k^2 - 400*x2_k + 2 , - 400*x1_k;
         - 400*x1_k, 200];
    %dx_k=-inv(H_k)*G_k;
    %A numerically better alternative to calc dx_k:
    %dx_k is solution of following system of lin eqs: H_k*dx_k = -G_k
    %which can be effectively solved using '\' operator:
    dx_k=-H_k\G_k;
    
    x_kp1=x_k+dx_k;
    x1_k=x_k(1);
    x2_k=x_k(2);
    x1_kp1=x_kp1(1);
    x2_kp1=x_kp1(2);
    %f_k=f(x1_k,x2_k,p1,p2,p3);
    f_k=100*(x2_k - x1_k^2)^2 + (1 - x1_k)^2;
    %f_kp1=f(x1_kp1,x2_kp1,p1,p2,p3);
    f_kp1=100*(x2_kp1 - x1_kp1^2)^2 + (1 - x1_kp1)^2;
    df=f_kp1-f_k
    x_k=x_kp1;
    if df < 0.00005
        break
    end
 end
disp('Optimal solution:')
x1_opt=x1_kp1
x2_opt=x2_kp1
%f_min=f(x1_opt,x2_opt,p1,p2,p3)
f_min=100*(x2_opt - x1_opt^2)^2 + (1 - x1_opt)^2