%% Clear and Init
clear; close all; clc;
global G L;
G = 9.81; %m/s
L = G; %m
th0 = 0; td0 = 0; thD = pi;
dt = 0.01;
tf = 40;
t = 0:dt:tf;

%% PID Control
clear theta thetaDot thetaD error;

kP = 3;
kI = 1.25;
kD = 4.5;

theta(1) = th0;
thetaDot(1) = td0;

thetaD = thD;


I = 0;
D = 0;
error(1) = thetaD - theta;

torque = (kP * error(1)) + (kI * I) + (kD * D);


for i = 2:length(t);
    
    res = PendulumPlant(theta(i-1), thetaDot(i-1), torque)*dt;
    K1 = res(1); L1 = res(2);
    
    res = PendulumPlant(theta(i-1) + K1/2, thetaDot(i-1) + L1/2, torque)*dt;
    K2 = res(1); L2 = res(2);
    
    res = PendulumPlant(theta(i-1) + K2/2, thetaDot(i-1) + L2/2, torque)*dt;
    K3 = res(1); L3 = res(2);
    
    res = PendulumPlant(theta(i-1) + K3  , thetaDot(i-1) + L3,torque)*dt;
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

thetaS(1) = th0;
thetaDotS(1) = td0;
thetaD = thD;

alpha = 2;
beta = 5;

S = @(theta, thetaDot) alpha*(theta - thetaD) + thetaDot;
u = @(S) -beta*sign(S);

torque = u(S(thetaS(1), thetaDotS(1)));


for i = 2:length(t);
    
    res = PendulumPlant(thetaS(i-1), thetaDotS(i-1), torque)*dt;
    K1 = res(1); L1 = res(2);
    
    res = PendulumPlant(thetaS(i-1) + K1/2, thetaDotS(i-1) + L1/2, torque)*dt;
    K2 = res(1); L2 = res(2);
    
    res = PendulumPlant(thetaS(i-1) + K2/2, thetaDotS(i-1) + L2/2, torque)*dt;
    K3 = res(1); L3 = res(2);
    
    res = PendulumPlant(thetaS(i-1) + K3  , thetaDotS(i-1) + L3,torque)*dt;
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
