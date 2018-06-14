function [cineq,ceq]=fun_constraints(x)
x1=x(1);
x2=x(2);

cineq = [];  % Compute nonlinear inequalities.
x2 >= x1+1; %Inequality constraint
 cineq =  -x2+x1+1;   % Compute nonlinear inequalities.
ceq = [];   % Compute nonlinear equalities.
end