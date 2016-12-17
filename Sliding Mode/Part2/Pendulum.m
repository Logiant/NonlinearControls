% a new version of the sliding mode control script!
clear; close all; clc;
%control parameters
lambda = 1; beta = 150; U = 0.05; %1, and 4
Kp = 500; Ki = 175; Kd = 250; %PID values
x1_0 = 0; x2_0 = 0; %initial conditions
%desired output
x1_d = @(t) pi/2;%1.5*sin(t/4) + pi/2;
x2_d = @(t) 0;%1.5/4*cos(t/4);%desired positions
%system parameters
g = 9.81; %gravity
L = 1.7; %(65)changing this has a small effect
m = 15;  %(30)changing this has a small effect
b = 2.0;  %(10)changing this has a large effect on PID
%system equations
x1_dot = @(x1, x2) x2;
x2_dot = @(x1, x2, u) -g/L*sin(x1) - b/(m*L^2)*x2 + u/m/L^2;
%approximated system parameters -- with +- error
L_hat = L;% + 0.1 * randn();
m_hat = m;% + 0.5 * randn();
b_hat = b;% + 0.5 * randn();
%control equations
s = @(x1, x2, t) lambda*(x1-x1_d(t)) + (x2 - x2_d(t)); % sliding surface
u_hat = @(x1, x2) m_hat*L_hat*g*sin(x1) + b_hat*x2 - m*L^2*lambda*(x2); %linearization
u = @(x1, x2, t) u_hat(x1, x2) - beta*sat(s(x1, x2, t),U); %control signal

% [x1, x2] = meshgrid([-2*pi:0.5:2*pi, -2*pi:0.5:2*pi]);
% 
% x1dot = x1_dot(x1, x2);
% x2dot = x2_dot(x1, x2,0);
% L = sqrt(x1dot.^2 + x2dot.^2);
% quiver(x1, x2, x1dot./L, x2dot./L)
% xlabel('\theta [rad]'); ylabel('\omega [rad/s]');
% hold on
%plot([-2*pi, 2*pi], [2*pi, -2*pi], '-k')
%figure()
%surf(s(x1, x2), x1, x2)

%% plot a trajectory
%sliding mode
x1S = x1_0; x2S = x2_0; %initial positions
tf = 60;
dt = 0.01;
t = 0:dt:tf; %simulation time

XDOT = @(x1, x2, u) [x1_dot(x1, x2), x2_dot(x1, x2, u)];
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
    U = u(x1S(i), x2S(i), t(i));
end

%PID
x1P = x1_0; x2P = x2_0; %initial positions

I = 0;
U = (Kp*(x1_d(0)-x1P)) + (Ki*I) + (Kd*(x2_d(0) - x2P));
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

    I = trapz(x1_d(t(i))-x1P)*dt;
    U = (Kp*(x1_d(t(i))-x1P(i))) + (Ki*I) + (Kd*(x2_d(t(i)) - x2P(i)));

end



hold on;
plot(x1S, x2S, 'r');%, x1P, x2P, 'b')
plot([pi/2 pi/2-1], [0, 1], 'k')
plot([pi/2 + 0.05 pi/2-1 + 0.05], [0, 1], '--g')
plot([pi/2 - 0.05 pi/2-1 - 0.05], [0, 1], '--g')
legend('System Response', 'Sliding Surface', '\Omega bounds')
%legend('Phase Portrait', 'Sliding Mode', 'PID')
xlabel('\theta [rad]'); ylabel('\omega [rad/s]');
figure(2);
plot(t, x1S, 'r', t, x1P, 'b', t, x1_d(t), '--k');
legend('SMC', 'PID', 'Desired Trajectory')
xlabel('Time [s]'); ylabel('\theta [rad]');
