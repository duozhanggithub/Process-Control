clear
clc

%Loads data from file into workspace.
load logfile1.txt;

%Define some constants
dt=0.02; %Sampling interval
Km=5; %5V/krpm
L=length(logfile1);%length of logfile

C = logfile1(:,2); %control signal
Sm = logfile1(:,3); %speed measurements as tachometer voltage
S = Sm/Km; %speed measurements as S [krpm]

%ds_dt is the known value
ds_dt = zeros(L-1,1);
%Calculate ds_dt using Euler forward
for i=1:L-1
    ds_dt(i) = (S(i+1)-S(i))/dt;
end

%Define the Regression matrix
Regression=[C(1:L-1) ones(L-1,1) -S(1:L-1)];

%Calculate the unknown values
b = ((Regression'*Regression)\Regression')*ds_dt;

K = b(1)/b(3)
L = b(2)/b(1)
T = 1/b(3)