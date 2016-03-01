%% Clear and Init
clear; close all; clc;
global L R C eSrc;
L = 1; %henry
R = 1; %ohm
C = 1; %farad
eSrc = 5; %volts
ec0 = 0; ecDot0 = 0; eD = eSrc/2;
dt = 0.01;
tf = 40;
t = 0:dt:tf;

%% PID Control
clear ec ecDot ecD error;

kP = 3;
kI = 1.25;
kD = 4.5;

ec(1) = ec0;
ecDot(1) = ecDot0;

ecD = eD;


I = 0;
D = 0;
error(1) = ecD - ec;

input = (kP * error(1)) + (kI * I) + (kD * D);


for i = 2:length(t);
    
    res = CircuitPlant(ec(i-1), ecDot(i-1), input)*dt;
    K1 = res(1); L1 = res(2);
    
    res = CircuitPlant(ec(i-1) + K1/2, ecDot(i-1) + L1/2, input)*dt;
    K2 = res(1); L2 = res(2);
    
    res = CircuitPlant(ec(i-1) + K2/2, ecDot(i-1) + L2/2, input)*dt;
    K3 = res(1); L3 = res(2);
    
    res = CircuitPlant(ec(i-1) + K3  , ecDot(i-1) + L3,input)*dt;
    K4 = res(1); L4 = res(2);

    ec(i) = ec(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    ecDot(i) = ecDot(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;

    
    error(i) = eD - ec(i);
    I = trapz(error) * dt;
    D = (error(i) - error(i-1))/dt;
    input = kP * error(i) + (kI * I) + (kD * D);
    
end

plot([0 tf], [eD eD], t, ec);
legend('Desired Position', 'PID')

%% Sliding Mode Control

clear ecS ecDotS ecD;

ecS(1) = ec0;
ecDotS(1) = ecDot0;
ecD = eD;

alpha = 2;
beta = 5;

S = @(ec, ecDot) alpha*(ec - ecD) + (ecDot - 0);
u = @(S) -beta*sign(S);

input = u(S(ecS(1), ecDotS(1)));


for i = 2:length(t);
    
    res = CircuitPlant(ecS(i-1), ecDotS(i-1), input)*dt;
    K1 = res(1); L1 = res(2);
    
    res = CircuitPlant(ecS(i-1) + K1/2, ecDotS(i-1) + L1/2, input)*dt;
    K2 = res(1); L2 = res(2);
    
    res = CircuitPlant(ecS(i-1) + K2/2, ecDotS(i-1) + L2/2, input)*dt;
    K3 = res(1); L3 = res(2);
    
    res = CircuitPlant(ecS(i-1) + K3  , ecDotS(i-1) + L3,input)*dt;
    K4 = res(1); L4 = res(2);

    ecS(i) = ecS(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    ecDotS(i) = ecDotS(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;

    
    input = u(S(ecS(i), ecDotS(i)));
    
end

if ~(exist('ec', 'var'))
    plot([0 tf], [ecD ecD], t, ecS);
    legend('Desired Position', 'Sliding mode')
else
    plot([0 tf], [ecD ecD], t, ecS, t, ec);
    legend('Desired Position', 'Sliding mode', 'PID')
end;
