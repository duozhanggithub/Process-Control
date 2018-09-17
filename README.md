# Process-Control 
check http://techteach.no/fag/process_control_nmbu_2018/ for all the details

Exercise 4

4_5.  Using CST: Discretization of a continuous-time model: See Exercise 4.3 in the exercise book. Discretize the given continuous-time state space model using the ZOH method with time-step 0.1 s. Calculate the eigenvalues of the resulting discrete-time model, and conclude about its stability property. Is the stability property the same as for the original continuous-time model ? 

4_6.  Symbolic linearization with Matlab Symbolic Toolbox: Linearize the nonlinear model (1.2) - (1.3) shown in the exercise book. (Tip: Use the jacobian function.) 
 
Optimization
The Rosenbrock optimization problem (“ROP”) is a “standard” optimization problem, cf. https://se.mathworks.com/help/optim/examples/banana-function-minimization.html.

4_7.  Grid search: Solve the ROP using the grid search method. The initial guess can be set to [-1.9,2] as in the above reference, and you may allow for a search of the optimal solution within -5 and 5 for both x1 and x2. 

4_8.  fmincon: The same as the above problem, but now by using fmincon. 

4_9.  Newton search: The same as the above problem, but now by implementing the Newton search method, from scratch. You may derive the gradient function and the Hessian function symbolically with the gradient() and the hessian() functions in the Symbolic Toolbox. 
Newton search: x_kp1 = x_k - inv[Hessian_f(x_k)]*Gradient_f(x_k), where “kp1” means “k plus 1”. 

4_9.  Grid search, with constraints: Same as the grid problem above, but now include the constraint x2 >= x1+1. 

4_10. fmincon, with constraints: Same as the fmincon problem above, but now include the constraint x2 >= x1+1. 

Exercise 5

5_1. Parameter estimation of a DC motor with least squares (LS) method: This web page (http://techteach.no/tekdok/dcmotor/) presents a DC motor. The web page includes some experimental data. Estimate K, T and L using the ordinary least squares method. As known data for the estimation, use control signal u [V] and speed S [krpm]. Do not use any special function in Matlab for the LS estimation, i.e., program from scratch the formulas that calculate the estimate. Finally, check, qualitatively, with a simulation if the model is good. 

5_2. Parameter estimation of an air heater using the grid optimization method: This web page (https://home.usn.no/finnh/air_heater/) presents an air heater. The web page includes some experimental data. Make a Matlab program which estimates the heater gain K_h, the time constant theta_t, the time-delay theta_d, and the environmental temperature T_env with the grid optimization method. Finally, run a simulation that (hopefully) demonstrates that the adapted model represents the real air heater well.
(Tip 1: Data can be loaded into the Matlab workspace with the load command. Tip 2: At each grid point, a simulation is run. Tip 3: A time-delay can be implementented with an array which contents are moved one array “cell” at each simulation iteration.)
 
5_3. Parameter estimation of the air heater using the nonlinear least squares (NLS) method: As Problem 2, but now use nonlinear least squares method implemented with fmincon() in Matlab.
(Tip: The objective function is calculated from a simulation of the model. In other words: At each iteration, the optimizer (fmincon) runs a simulation.)

5_5. Subspace identification of the air heater:  Try to identify an input/output model (a discrete-time state space model) of the air heater using subspace identification (n4sid() in Matlab). Check if the model is good.
(Tip 1: The process contains a time-delay of some seconds. This may cause problems for the identification with n4sid since the model form assumed by n4sid does not directly include any time-delay term. Cf. the comments in the lecture about this.
Tip 2: Matlab script for subspace identification of DC motor using these experiemental data, demonstrated in the lecture 18th March 2018.) 

Exercise 6

6_1 Kalman Filter
6_2 Moving Horizon Estimation (MHE)
6_3 Model-predictive Control (MPC):
