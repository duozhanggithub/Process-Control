%Finn Haugen (finn@techteach.no)
%21.2 2018
%----------------------------------------------------------
disp('-------------------')
disp('Grid search method to solve an')
disp('optimization problem:')
disp('min_x f(x)')
disp('s.t.')
disp('model: f=100*(x2-x1^2)^2+(1-x1)^2')
%disp('params: p1 = 1, p2 = 2, p3 = 3')
disp('constraints:')
disp('-5 <= x1 <= 5')
disp('-5 <= x2 <= 5')
disp('x2 >= x1+1')
%----------------------------------------------------------
clear all 
close all 
format compact

%Model parameters:
%p1 = 1;
%p2 = 2;
%p3 = 3;
%Initialization:
x1_min=-5;x1_max=5;N_x1=100;
x1_array=linspace(x1_min,x1_max,N_x1);

x2_min=-5;x2_max=5;N_x2=100;
x2_array=linspace(x2_min,x2_max,N_x2);

f_min=inf;
x1_opt=-1.9; %initial guess
x2_opt=2; %initial guess

tic;

for k_x1=1:length(x1_array)
    x1=x1_array(k_x1);

    for k_x2=1:length(x2_array)
        x2=x2_array(k_x2);

        %Objective function:
        f=100*(x2-x1^2)^2+(1-x1)^2; %the Rosenbrock funtion

         %Constraint:
         if x2 >= x1+1,
              f=inf;
         end

        %Improving the previous solution:
         if f <= f_min,
             f_min=f;
             x1_opt=x1;
             x2_opt=x2;
         end

         %Storing objective function values for later plotting:
         f_matrix(k_x1,k_x2)=f;

    end
end

%--------------------------------------------------
%Finding the optimal solution:

disp('-------------------')
disp('Optimal solution:')

f_min
x1_opt
x2_opt

%--------------------------------------------------
%Calculation of execution times:

disp('-------------------')
disp('Execution times of running the nested for-loops:')

dt_elapsed_if=toc
dt_elapsed_per_cycle_if=toc/(N_x1*N_x2)
N_tot=N_x1*N_x2

%--------------------------------------------------
%3D plot of the objective function:

h=figure; %Getting figure handle
fig_posleft=8;fig_posbottom=1.5;fig_width=18;fig_height=16;
fig_pos_size_1=[fig_posleft,fig_posbottom,fig_width,fig_height];

set(gcf,'Units','centimeters','Position',fig_pos_size_1);
figtext='Optim plot';
set(gcf,'Name',figtext,'NumberTitle','on')

mesh(x1_array,x2_array,f_matrix');

azimut=-20;elevation=20;
view(azimut,elevation);
xlabel('x1')
ylabel('x2')
zlabel('f')

%Plotting a vertical line of the optimal solution:
N_line=10;
line_x1=linspace(x1_opt,x1_opt,N_line); 
line_x2=linspace(x2_opt,x2_opt,N_line); 
line_f=linspace(0,f_min,N_line);
point1_f=linspace(f_min,f_min,N_line);
point2_f=linspace(0,0,N_line);

hold on
plot3(line_x1,line_x2,point1_f,'*r',...
    line_x1,line_x2,point2_f,'or',...
    line_x1,line_x2,line_f,'r');
hold off

%Saving the plot as a PDF file:
%saveas(h,'fig1','pdf')

return

%--------------------------------------------------
%Testing the result by applying positive and negative
%perturbations to the optimal solution (not caring about
%the constraints here):
disp('-------------------')
disp('Perturbation test for optimality:')
disp('(The test may fail if optim solution is on a constraint.)')

pert_x1=0
pert_x2=0
x1=x1_optim+pert_x1;
x2=x2_optim+pert_x2;
x1_pert=x1
x2_pert=x2
f_pert=100*(x2-x1^2)^2+(1-x1)^2
f_min
is_constraint_satisfied=(x2 > x1+1)