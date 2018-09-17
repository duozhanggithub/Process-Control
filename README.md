# Process-Control

4-5.  Using CST: Discretization of a continuous-time model: See Exercise 4.3 in the exercise book. Discretize the given continuous-time state space model using the ZOH method with time-step 0.1 s. Calculate the eigenvalues of the resulting discrete-time model, and conclude about its stability property. Is the stability property the same as for the original continuous-time model ? (Cf. Problem 3 above.) 

4-6.  Symbolic linearization with Matlab Symbolic Toolbox: Linearize the nonlinear model (1.2) - (1.3) shown in the exercise book. (Tip: Use the jacobian function.) 
 

Optimization
The Rosenbrock optimization problem (“ROP”) is a “standard” optimization problem, cf. https://se.mathworks.com/help/optim/examples/banana-function-minimization.html.

4-7.  Grid search: Solve the ROP using the grid search method. The initial guess can be set to [-1.9,2] as in the above reference, and you may allow for a search of the optimal solution within -5 and 5 for both x1 and x2. 

4-8.  fmincon: The same as the above problem, but now by using fmincon. (Template for solution, similar to script presented in lecture 22 Feb.) 

4-9.  Newton search: The same as the above problem, but now by implementing the Newton search method, from scratch. You may derive the gradient function and the Hessian function symbolically with the gradient() and the hessian() functions in the Symbolic Toolbox. 
Newton search: x_kp1 = x_k - inv[Hessian_f(x_k)]*Gradient_f(x_k), where “kp1” means “k plus 1”. 

4-9.  Grid search, with constraints: Same as the grid problem above, but now include the constraint x2 >= x1+1. 

4-10. fmincon, with constraints: Same as the fmincon problem above, but now include the constraint x2 >= x1+1. 
