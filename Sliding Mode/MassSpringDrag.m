% a new version of the sliding mode control script!
clear; close all; clc;
%control parameters
lambda = 3; beta = 4;
%desired output
x1_d = 1; x2_d = 0; x2dot_d = 0; %stay at x1d
%system parameters
Cd = 65; %(65)changing this has a small effect
m = 30;  %(30)changing this has a small effect
k = 10;  %(10)changing this has a large effect on PID
%system equations
x1_dot = @(x1, x2) x2;
x2_dot = @(x1, x2, u) -Cd/m*x2.*abs(x2) - k/m*x1 + u;
%helper equation
getRandN = @() rand()/4 + 1 - 1/8; % uniform dist in R = (0.75, 1.25)
%approximated system parameters -- with +- 25% error
Cd_hat = Cd * getRandN();
m_hat = m * getRandN();
k_hat = k * getRandN();
%control equations
s = @(x1, x2) lambda*(x1_d-x1) + (x2_d-x2); % sliding surface
u_hat = @(x1, x2) Cd_hat/m_hat*abs(x2).*x2 + k_hat/m_hat*x1 + lambda*(x2_d - x2) + x2dot_d; %linearization
u = @(x1, x2) beta*sign(s(x1, x2)) + u_hat(x1, x2); %control signal

[x1, x2] = meshgrid([-1.5:0.05:1.5, -1.5:0.05:1.5]);

x1dot = x1_dot(x1, x2);
x2dot = x2_dot(x1, x2, u(x1, x2));
quiver(x1, x2, x1dot, x2dot)


%% plot a trajectory
%sliding mode
x1S = -1; x2S = 0; %initial positions
tf = 50;
dt = 0.01;
t = 0:dt:tf; %simulation time

XDOT = @(x1, x2, u) [x1_dot(x1, x2), x2_dot(x1, x2, u)];
U = u(x1S, x2S);
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
    U = u(x1S(i), x2S(i));
end

%PID
x1P = -1; x2P = 0; %initial positions
Kp = 10; Ki = 0.25; Kd = 5;
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
legend('Phase Portrait', 'Sliding Mode', 'PID')
xlabel('Position [m]'); ylabel('Velocity [m/s]');
figure(2);
plot(t, x1S, 'r', t, x1P, 'b', [0 tf], [x1_d, x1_d]);
legend('Sliding Mode', 'PID', 'Desired Position')
xlabel('Time [s]'); ylabel('Position [m]');
