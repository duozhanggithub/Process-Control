clear
clc

%Loads data from file into workspace.
load airheater_logfile.txt;
Ts=0.1; %Sampling interval
L=length(airheater_logfile);%length of logfile
F=0.7;%First fraction (portion) of logfile to be used for model adaption:
N=floor(L*F); %N is number of samples used for estimation.

 %Rest used for visual validation.
t_estim=Ts*[1:N]';
u_estim=airheater_logfile(1:N,2);
y_estim=airheater_logfile(1:N,3);

t_valid=Ts*[N+1:L]';
u_valid=airheater_logfile(N+1:L,2);
y_valid=airheater_logfile(N+1:L,3);

t_total=Ts*[1:L]';
u_total=airheater_logfile(1:L,2);
y_total=airheater_logfile(1:L,3);

%modelorder='best';%Defines order of estimated model.
modelorder=1;%Defines order of estimated model.

%Estimation of model. Model is on internal theta-format:
[model_est, x0]=n4sid(iddata(y_estim,u_estim,Ts),modelorder,'InputDelay',4); 

% model_est=n4sid([y_estim u_estim],modelorder); 
%th2tf-function calculates numerator and denominator coeff. arrays
%in z-transfer function:
[num,den]=th2tf(model_est);
H_disc=tf(num,den,Ts) %Generates an LTI-model from z-transf func.
y_sim=lsim(H_disc,u_total,t_total);

%Plots:
h=figure; %Getting figure handle
fig_posleft=8;fig_posbottom=1.5;fig_width=24;fig_height=20;
fig_pos_size_1=[fig_posleft,fig_posbottom,fig_width,fig_height];

set(gcf,'Units','centimeters','Position',fig_pos_size_1);
figtext='Estimation of model params of DC motor';
set(gcf,'Name',figtext,'NumberTitle','on')

subplot(2,1,1)
figure(1)
plot(t_total,y_sim,'r',t_total,y_total,'b',t_estim,t_estim*0-4,'g*');
title('Real y (blue). Simulated y with adapted model (red). Green: Interval for adaption.')
grid minor
ylim([-4,40]);
ylabel('[V]');xlabel('t [s]')

subplot(2,1,2)
plot(t_total,u_total,'b',t_estim,t_estim*0-4,'g*');
title('Applied control sigal, u, to both real and simulated process. Green: Interval for adaptation.')
grid minor
ylim([-4,4]);
ylabel('[V]');xlabel('t [s]')