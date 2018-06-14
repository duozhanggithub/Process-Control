%Finn Haugen (finn@techteach.no)
%21.2 2018
%----------------------------------------------------------
disp('-------------------')
disp('Using fmincon() in Matlab solve an optimization')
disp('(Nonlinear Programming, NLP) problem:')
disp('min_x f(x)')
disp('s.t.')
disp('model: f=100*(x(2) - x(1)^2)^2 + (1 - x(1))^2')
%disp('params: p1 = 1, p2 = 2, p3 = 3')
disp('constraints:')
disp('-5 <= x1 <= 5') %Variable constraint
disp('-5 <= x2 <= 5') %Variable constraint
%disp('x2 >= x1+1') %Inequality constraint
%----------------------------------------------------------
format compact
clear all
close all
%--------------------------------------------------
% Model params:
%p1=1;
%p2=2;
%p3=3;
%params_model.p1=p1;%params_model is a struct. p1 is a field.
%params_model.p2=p2;
%params_model.p3=p3;

%Contraints:
x1_min=-5;x1_max=5;
x2_min=-5;x2_max=5;
x_lb=[x1_min,x2_min];
x_ub=[x1_max,x2_max];

%--------------------------------------------------
%fmincon:
x_guess=[-1.9,2];
Aineq=[]; Bineq=[]; Aeq=[]; Beq=[];

fun = @(x)(100*(x(2) - x(1)^2)^2 + (1 - x(1))^2);%Local function
%fun_constraints_handle = @(x)fun_constraints(x,params_model);%Local function
optim_options=optimset('Display','on');
%optim_options=optimset('Algorithm','sqp','Display','on');
[x_opt,fval,exitflag,output,lambda,grad,hessian] =...
fmincon(fun,x_guess,Aineq,Bineq,Aeq,Beq,x_lb,x_ub,[],optim_options);

%--------------------------------------------------
%Displaying the optimal solution:
disp('-------------------')
disp('Optimal solution:')
output
fval
x1_opt=x_opt(1)
x2_opt=x_opt(2)

%--------------------------------------------------
%Applying the optimal solution:
f_min=100*(x2_opt - x1_opt^2)^2 + (1 - x1_opt)^2