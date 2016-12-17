%% Clear and Init
clear; close all; clc;
%---system parameters---%
Cd = 12; %m/s    - Tuned at 4
m = 15; %m      - Tuned at 25
k = 20; %kg     - Tuned at 50
%---temporal parameters---%
x0 = 0; xd0 = 0; xD = 1.5; % Tuned at (0, 0, 0.5)
dt = 0.01;
tf = 15;
t = 0:dt:tf;
%---system equations---%
x1dot = @(x1, x2) x2;
x2dot = @(x1, x2, Fin) -Cd/m*x2*abs(x2) -k/m*x1 + Fin/m;
XDOT = @(x1, x2, Fin) [x1dot(x1, x2), x2dot(x1, x2, Fin)];

%% PID Control
clear x1 x2 x1D error;
%---controller parameters---%
kP = 750;
kI = 100;
kD = 500;

x1(1) = x0;
x2(1) = xd0;


I = 0;
D = 0;
error(1) = xD - x1;

F = (kP * error(1)) + (kI * I) + (kD * D);


for i = 2:length(t);
    
    res = XDOT(x1(i-1), x2(i-1), F)*dt;
    K1 = res(1); L1 = res(2);
    
    res = XDOT(x1(i-1) + K1/2, x2(i-1) + L1/2, F)*dt;
    K2 = res(1); L2 = res(2);
    
    res = XDOT(x1(i-1) + K2/2, x2(i-1) + L2/2, F)*dt;
    K3 = res(1); L3 = res(2);
    
    res = XDOT(x1(i-1) + K3  , x2(i-1) + L3,F)*dt;
    K4 = res(1); L4 = res(2);

    x1(i) = x1(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    x2(i) = x2(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;

    
    error(i) = xD - x1(i);
    I = trapz(error) * dt;
    D = -x2(i);
    F = kP * error(i) + (kI * I) + (kD * D);
    
end

plot([0 tf], [xD xD], t, x1);
legend('Desired Position', 'PID')

%% Sliding Mode Control

clear x1S x2S x1D;
%---controller parameters---%
lambda = 5;
beta = 400;


x1S(1) = x0;
x2S(1) = xd0;
x1D = xD;

S = @(x1, x2) lambda*(x1D - x1) - x2;
u = @(S) beta*sign(S);

F = u(S(x1S(1), x2S(1)));


for i = 2:length(t);
    
    res = XDOT(x1S(i-1), x2S(i-1), F)*dt;
    K1 = res(1); L1 = res(2);
    
    res = XDOT(x1S(i-1) + K1/2, x2S(i-1) + L1/2, F)*dt;
    K2 = res(1); L2 = res(2);
    
    res = XDOT(x1S(i-1) + K2/2, x2S(i-1) + L2/2, F)*dt;
    K3 = res(1); L3 = res(2);
    
    res = XDOT(x1S(i-1) + K3  , x2S(i-1) + L3,F)*dt;
    K4 = res(1); L4 = res(2);

    x1S(i) = x1S(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    x2S(i) = x2S(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;

    
    F = u(S(x1S(i), x2S(i)));
    
end

if ~(exist('x1', 'var'))
    plot([0 tf], [x1D x1D], t, x1S);
    legend('Desired Position', 'Sliding mode')
else
    plot([0 tf], [x1D x1D], t, x1S, t, x1);
    legend('Desired Position', 'Sliding mode', 'PID')
end;
axis square
xlabel('Time [s]')
ylabel('Position [m]')