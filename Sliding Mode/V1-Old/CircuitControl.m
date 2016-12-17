%% Clear and Init
clear; close all; clc;
%---system parameters---%
L = 0.001; %H   - Tuned at 0.001
R = 500; %Ohm   - Tuned at 500
C = 1e-6; %F    - Tuned at 1e-6
eSrc = 5; %V    - Tuned at 5
%---temporal parameters---%
ec0 = 0; ecDot0 = 0;eD = eSrc/4; %tuned at (0, 0, esrc/2)
dt = 0.000001;
tf = 0.01;
t = 0:dt:tf;
%---system equations---%
x1dot = @(x1, x2) x2;
x2dot = @(x1, x2, on) ((eSrc*on) - x2*C*R - x1)/(L*C);
XDOT = @(x1, x2, on) [x1dot(x1, x2), x2dot(x1, x2, on)];

%% PID Control
clear ec ecDot ecD error;
%---controller parameters---%
kP = 3;
kI = 1;
kD = 0.5;

ec(1) = ec0;
ecDot(1) = ecDot0;

ecD = eD;


I = 0;
D = 0;
error(1) = ecD - ec;

input = (kP * error(1)) + (kI * I) + (kD * D);


for i = 2:length(t);
    
    if input > 0
        input = 1;
    else 
        input = 0;
    end
    
    res = XDOT(ec(i-1), ecDot(i-1), input)*dt;
    K1 = res(1); L1 = res(2);
    
    res = XDOT(ec(i-1) + K1/2, ecDot(i-1) + L1/2, input)*dt;
    K2 = res(1); L2 = res(2);
    
    res = XDOT(ec(i-1) + K2/2, ecDot(i-1) + L2/2, input)*dt;
    K3 = res(1); L3 = res(2);
    
    res = XDOT(ec(i-1) + K3  , ecDot(i-1) + L3,input)*dt;
    K4 = res(1); L4 = res(2);

    ec(i) = ec(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    ecDot(i) = ecDot(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;

    
    error(i) = ecD - ec(i);
    I = trapz(error) * dt;
    D = -ecDot(i);
    input = kP * error(i) + (kI * I) + (kD * D);
    
end

plot([0 tf], [eD eD], t, ec);
legend('Desired Position', 'PID')

%% Sliding Mode Control

clear ecS ecDotS ecD;
%---controller parameters---%
lambda = 2;
beta = 1;

ecS(1) = ec0;
ecDotS(1) = ecDot0;
ecD = eD;



S = @(ec, ecDot) lambda*(ecD - ec) - (ecDot);
u = @(x1, x2) beta*sign(S(x1, x2));

input = u(ecS(1), ecDotS(1));


for i = 2:length(t);
    
    if input > 0
        input = 1;
    else 
        input = 0;
    end
    
    
    res = XDOT(ecS(i-1), ecDotS(i-1), input)*dt;
    K1 = res(1); L1 = res(2);
    
    res = XDOT(ecS(i-1) + K1/2, ecDotS(i-1) + L1/2, input)*dt;
    K2 = res(1); L2 = res(2);
    
    res = XDOT(ecS(i-1) + K2/2, ecDotS(i-1) + L2/2, input)*dt;
    K3 = res(1); L3 = res(2);
    
    res = XDOT(ecS(i-1) + K3  , ecDotS(i-1) + L3,input)*dt;
    K4 = res(1); L4 = res(2);

    ecS(i) = ecS(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    ecDotS(i) = ecDotS(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;

    
    input = u(ecS(i), ecDotS(i));
    
end

if ~(exist('ec', 'var'))
    plot([0 tf], [ecD ecD], t, ecS);
    legend('Desired Position', 'Sliding mode')
else
    plot([0 tf], [ecD ecD], t, ecS, t, ec);
    legend('Desired Position', 'Sliding mode', 'PID')
end;
axis square
xlabel('Time [s]')
ylabel('Capacitor Voltage [v]')
