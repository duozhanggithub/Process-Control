%----------------------------------------------------------
%Parameter estimation of a DC motor with n4sid subspace identification.

%Info about motor: http://techteach.no/tekdok/dcmotor/

%Finn Haugen (finn.haugen@usn.no)
%University College of Southeast Norway
%14 Dec 2017
%----------------------------------------------------------

clear all
close all
%Loads data from file into workspace.
load logfile1.txt;
Ts=0.02; %Sampling interval
L=length(logfile1);%(Matrix name becomes same as logfile name.)
F=0.5;%First fraction (portion) of logfile to be used for model adaption:
N=floor(L*F); %N is number of samples used for estimation.
 %Rest used for visual validation.
t_estim=Ts*[1:N]';
u_estim=logfile1(1:N,2);
y_estim=logfile1(1:N,3);
t_valid=Ts*[N+1:L]';
u_valid=logfile1(N+1:L,2);
y_valid=logfile1(N+1:L,3);
t_total=Ts*[1:L]';
u_total=logfile1(1:L,2);
y_total=logfile1(1:L,3);
% modelorder='best';%Defines order of estimated model.
modelorder=1;%Defines order of estimated model.
%Estimation of model. Model is on internal theta-format:
model_est=n4sid(iddata(y_estim,u_estim,Ts),modelorder); 
% model_est=n4sid([y_estim u_estim],modelorder); 
%th2tf-function calculates numerator and denominator coeff. arrays
%in z-transfer function:
[num,den]=th2tf(model_est);
H_disc=tf(num,den,Ts); %Generates an LTI-model from z-transf func.
y_sim=lsim(H_disc,u_total,t_total);%Simulates with u_valid as input.

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
ylim([-4,4]);
ylabel('[V]');xlabel('t [s]')

subplot(2,1,2)
plot(t_total,u_total,'b',t_estim,t_estim*0-4,'g*');
title('Applied control sigal, u, to both real and simulated process. Green: Interval for adaptation.')
grid minor
ylim([-4,4]);
ylabel('[V]');xlabel('t [s]')

H_cont=d2c(H_disc,'zoh') %Converts to s-transfer function.
[num_H_cont,denum_H_cont]=tfdata(H_cont);
b1=num_H_cont{1}(1);
b0=num_H_cont{1}(2);
a1=denum_H_cont{1}(1);
a0=denum_H_cont{1}(2);
K=b0/a0
T=1/a0
