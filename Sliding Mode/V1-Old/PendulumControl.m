%% Clear and Init
clear; close all; clc;
%---system parameters---%
g = 9.81;   %m/s   - Tuned at 9.81
L = 1;      %m     - Tuned at 1
m = 0.5;    %kg    - Tuned at 0.5
b = 1;      %N-m/s - Tuned at 1
%---temporal parameters---%
th0 = 0; td0 = 0; thD = 2.5; %Tuned at (0, 0, 0.5)
dt = 0.01;
tf = 10;
t = 0:dt:tf;
%---system equations---%
x1dot = @(x1, x2) x2;
x2dot = @(x1, x2, torque) -g/L*sin(x1) -b*x2 + torque/(m*L^2);
XDOT = @(x1, x2, torque) [x1dot(x1, x2), x2dot(x1, x2, torque)];

%% PID Control
clear theta thetaDot thetaD error;
%---controller parameters---%
kP = 15;
kI = 10;
kD = 3;

theta(1) = th0;
thetaDot(1) = td0;

thetaD = thD;


I = 0;
D = 0;
error(1) = thetaD - theta;

torque = (kP * error(1)) + (kI * I) + (kD * D);


for i = 2:length(t);
    
    res = XDOT(theta(i-1), thetaDot(i-1), torque)*dt;
    K1 = res(1); L1 = res(2);
    
    res = XDOT(theta(i-1) + K1/2, thetaDot(i-1) + L1/2, torque)*dt;
    K2 = res(1); L2 = res(2);
    
    res = XDOT(theta(i-1) + K2/2, thetaDot(i-1) + L2/2, torque)*dt;
    K3 = res(1); L3 = res(2);
    
    res = XDOT(theta(i-1) + K3  , thetaDot(i-1) + L3,torque)*dt;
    K4 = res(1); L4 = res(2);

    theta(i) = theta(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    thetaDot(i) = thetaDot(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;

    
    error(i) = thetaD - theta(i);
    I = trapz(error) * dt;
    D = (error(i) - error(i-1))/dt;
    torque = kP * error(i) + (kI * I) + (kD * D);
    
end

plot([0 tf], [thetaD thetaD], t, theta);
legend('Desired Position', 'PID')

%% Sliding Mode Control

clear thetaS thetaDotS thetaD;
%---controller parameters---%
lambda = 10;
beta = 10;


thetaS(1) = th0;
thetaDotS(1) = td0;
thetaD = thD;



S = @(theta, thetaDot) lambda*(thetaD - theta) - thetaDot;
u = @(S) beta*sign(S);

torque = u(S(thetaS(1), thetaDotS(1)));


for i = 2:length(t);
    
    res = XDOT(thetaS(i-1), thetaDotS(i-1), torque)*dt;
    K1 = res(1); L1 = res(2);
    
    res = XDOT(thetaS(i-1) + K1/2, thetaDotS(i-1) + L1/2, torque)*dt;
    K2 = res(1); L2 = res(2);
    
    res = XDOT(thetaS(i-1) + K2/2, thetaDotS(i-1) + L2/2, torque)*dt;
    K3 = res(1); L3 = res(2);
    
    res = XDOT(thetaS(i-1) + K3  , thetaDotS(i-1) + L3,torque)*dt;
    K4 = res(1); L4 = res(2);

    thetaS(i) = thetaS(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    thetaDotS(i) = thetaDotS(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;

    
    torque = u(S(thetaS(i), thetaDotS(i)));
    
end

if ~(exist('theta', 'var'))
    plot([0 tf], [thetaD thetaD], t, thetaS);
    legend('Desired Position', 'Sliding mode')
else
    plot([0 tf], [thetaD thetaD], t, thetaS, t, theta);
    legend('Desired Position', 'Sliding mode', 'PID')
end;

axis square
xlabel('Time [s]')
ylabel('Pendulum Angle [rad]')
