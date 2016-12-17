% a new version of the sliding mode control script!
clear; close all; clc;
%control parameters
lambda = 1; k1 = 7.5; k2 = 1;
%desired output
x1_d = 1; x2_d = 0; x2dot_d = 0; %stay at x1d
%system parameters
g = 9.81; %gravity
L = 0.75; %(65)changing this has a small effect
m = 5;  %(30)changing this has a small effect
b = 1.5;  %(10)changing this has a large effect on PID
%system equations
x1_dot = @(x1, x2) x2;
x2_dot = @(x1, x2, u) -g/L*sin(x1) - b*x2 + u;
%helper equation
getRandN = @() rand()/4 + 1 - 1/8; % uniform dist in R = (0.75, 1.25)
%approximated system parameters -- with +- 25% error
L_hat = L * getRandN();
m_hat = m * getRandN();
b_hat = b * getRandN();
%control equations
s = @(x1, x2) lambda*(x1_d-x1) + (x2_d-x2); % sliding surface
u_hat = @(x1, x2) g/L_hat*sin(x1) + b_hat/(m_hat*L_hat^2)*x2 + lambda*(x2_d - x2) + x2dot_d; %linearization
u = @(x1, x2, I) k1*sqrt(abs(s(x1,x2))).*sign(s(x1, x2)) ...
                + k2*I + ...
              + u_hat(x1, x2); %control signal

[x1, x2] = meshgrid([-1.5:0.05:1.5, -1.5:0.05:1.5]);

x1dot = x1_dot(x1, x2);
x2dot = x2_dot(x1, x2, u(x1, x2, 0));

L = sqrt(x1dot.^2 + x2dot.^2);

quiver(x1, x2, x1dot./L, x2dot./L)


%% plot a trajectory
%sliding mode
x1S = -1; x2S = 0; %initial positions
tf = 20;
dt = 0.01;
t = 0:dt:tf; %simulation time

XDOT = @(x1, x2, u) [x1_dot(x1, x2), x2_dot(x1, x2, u)];
Is = 0;
U = u(x1S, x2S, 0);
for i = 2:length(t);
    
    res = XDOT(x1S(i-1), x2S(i-1), U)*dt;
    K1 = res(1); L1 = res(2);
    res = XDOT(x1S(i-1) + K1/2, x2S(i-1) + L1/2, U)*dt;
    K2 = res(1); L2 = res(2);
    res = XDOT(x1S(i-1) + K2/2, x2S(i-1) + L2/2, U)*dt;
    K3 = res(1); L3 = res(2);
    res = XDOT(x1S(i-1) + K3  , x2S(i-1) + L3,U)*dt;
    K4 = res(1); L4 = res(2);
    x1S(i) = x1S(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    x2S(i) = x2S(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;
    
    Is = Is + sign(s(x1S(i), x2S(i)))*dt;
    U = u(x1S(i), x2S(i), Is);
end

%PID
x1P = -1; x2P = 0; %initial positions
Kp = 10; Ki = 10; Kd = 5;
I = 0;
U = (Kp*(x1_d-x1P)) + (Ki*I) + (Kd*(x2_d - x2P));
for i = 2:length(t);
    
    res = XDOT(x1P(i-1), x2P(i-1), U)*dt;
    K1 = res(1); L1 = res(2);
    res = XDOT(x1P(i-1) + K1/2, x2P(i-1) + L1/2, U)*dt;
    K2 = res(1); L2 = res(2);
    res = XDOT(x1P(i-1) + K2/2, x2P(i-1) + L2/2, U)*dt;
    K3 = res(1); L3 = res(2);
    res = XDOT(x1P(i-1) + K3  , x2P(i-1) + L3,U)*dt;
    K4 = res(1); L4 = res(2);
    x1P(i) = x1P(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    x2P(i) = x2P(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;

    I = trapz(x1_d-x1P)*dt;
    U = (Kp*(x1_d-x1P(i))) + (Ki*I) + (Kd*(x2_d - x2P(i)));

end



hold on;
plot(x1S, x2S, 'r', x1P, x2P, 'b')
legend('Phase Portrait', 'Super Twisting Mode', 'PID')
xlabel('Position [m]'); ylabel('Velocity [m/s]');
figure(2);
plot(t, x1S, 'r', t, x1P, 'b', [0 tf], [x1_d, x1_d]);
legend('Super Twisting Mode', 'PID', 'Desired Position')
xlabel('Time [s]'); ylabel('Position [m]');
