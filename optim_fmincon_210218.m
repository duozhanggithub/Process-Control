%Finn Haugen (finn@techteach.no)
%21.2 2018
%----------------------------------------------------------
disp('-------------------')
disp('Using fmincon() in Matlab solve an optimization')
disp('(Nonlinear Programming, NLP) problem:')
disp('min_x f(x)')
disp('s.t.')
disp('model: f=(x1-p1)^2+(x2-p2)^2+p3')
disp('params: p1 = 1, p2 = 2, p3 = 3')
disp('constraints:')
disp('0 <= x1 <= 3') %Variable constraint
disp('0 <= x2 <= 4') %Variable constraint
%disp('x2 >= x1+1') %Inequality constraint
%----------------------------------------------------------
format compact
clear all
close all
%--------------------------------------------------
% Model params:
p1=1;
p2=2;
p3=3;
params_model.p1=p1;%params_model is a struct. p1 is a field.
params_model.p2=p2;
params_model.p3=p3;

%Contraints:
x1_min=0;x1_max=3;
x2_min=0;x2_max=4;
x_lb=[x1_min,x2_min];
x_ub=[x1_max,x2_max];

%--------------------------------------------------
%fmincon:
x_guess=[4,5];
Aineq=[]; Bineq=[]; Aeq=[]; Beq=[];

fun_objective_handle = @(x)fun_objective(x,params_model);%Local function
fun_constraints_handle = @(x)fun_constraints(x,params_model);%Local function
optim_options=optimset('Display','on');
%optim_options=optimset('Algorithm','sqp','Display','on');
[x_opt,fval,exitflag,output,lambda,grad,hessian] =...
fmincon(fun_objective_handle,x_guess,Aineq,Bineq,Aeq,Beq,x_lb,x_ub,...
    fun_constraints_handle,optim_options);

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
f_min=(x1_opt-p1)^2+(x2_opt-p2)^2+p3

%--------------------------------------------------
%Defining local functions:

function f = fun_objective(x,params_model)
x1=x(1);
x2=x(2);
p1=params_model.p1;
p2=params_model.p2;
p3=params_model.p3;
f=(x1-p1)^2+(x2-p2)^2+p3;
end

function [cineq,ceq]=fun_constraints(x,params_model)
x1=x(1);
x2=x(2);
p1=params_model.p1;
p2=params_model.p2;
p3=params_model.p3;

cineq = [];  % Compute nonlinear inequalities.
%x2 >= x1+2 %Inequality constraint
% cineq =  -x2+x1+2;   % Compute nonlinear inequalities.
ceq = [];   % Compute nonlinear equalities.
end

