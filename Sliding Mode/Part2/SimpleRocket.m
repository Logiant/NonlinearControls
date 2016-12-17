%rocket maaaan
clear; close all; clc;
%controller parameters
beta = 0.005; lambda = 1;
thresh = 0.8;
%rocket parameters
rho = 1.225; % kg/m3 - air density
rho_hat = rho + 0.001*randn();
Cd = 0.3392; %drag coeff.
Cd_hat = Cd + 0.0002*randn();
A = 2.010E-3;% m2 - cross sectional area
A_hat = A + 0.00001*randn();
g0 = 9.81; % m/s2 - gravity of earth
g0_hat = 9.81 + 0.01*randn();
g = g0;
g_hat = g + 0.01*randn();
Isp = 350; % s - specific impulse
Isp_hat = Isp + 10*randn();
% Kinematic parameters
x_d = 120; %m
x0 = 0; v0 = 0;
%dynamic equations
s = @(x, v) lambda*(x-x_d) + v;
y_dot = @(y, v, m, mdot) v;
v_dot = @(y, v, m, mdot) -(1/2*rho*Cd*A/m)*v.*abs(v) - g + (g0/m*Isp-v/m).*mdot;
m_dot = @(y, v, m) rho_hat*v*abs(v)*Cd_hat*A_hat/(2*g0_hat*Isp_hat-v) ...
                   +m*g/(g0_hat*Isp_hat-v)-lambda*m*v/(g0_hat*Isp_hat-v) ...
                   -beta*sat(s(y, v), 1);

%temporal parameters
tf = 30; dt = 0.005; t = 0:dt:tf;
%initial conditions
i = 1;
x(i) = x0;
v(i) = v0;
m = 0.273; % kg - dry mass
mf(i) = 0.2; %kg - fuel
mdot(i) = 0;
i_cross = 0;
for i = 2:length(t)

    x(i) = x(i-1) + y_dot(x(i-1), v(i-1), mf(i-1)+m, mdot(i-1))*dt;
    x(i) = max(x(i), 0); %don't penetrate the ground
    v(i) = v(i-1) + v_dot(x(i-1), v(i-1), mf(i-1)+m, mdot(i-1))*dt;
    v(i) = max(v(i), -x(i)/dt);  %don't drop below ground
    mdot(i) = m_dot(x(i-1), v(i-1), mf(i-1)+m);
    mdot(i) = min(mf(i-1)/dt, mdot(i)); %don't go below 0 fuel
    if (mf(i-1) <= 0)
        mdot(i) = 0;
    end
    mf(i) = mf(i-1) - abs(mdot(i-1))*dt;
    if abs(s(x(i-1), v(i-1))) < thresh && i_cross == 0
        i_cross = i-1;
    end
end
if (i_cross > 0)
plot(t, x, [0 tf], [x_d, x_d], t(i_cross), x(i_cross), 'or')
else
plot(t, x, [0 tf], [x_d, x_d])
end
xlabel('Time [s]'); ylabel('Altitude [m]');
legend('Response', 'Desired Position', 'Transition')
figure
if (i_cross > 0)
plot(x, v, x_d, 0, 'xk', x(i_cross), v(i_cross), 'or')
else
plot(x, v, x_d, 0, 'xk')
end
xlabel('Altitude [m]'); ylabel('Velocity [m/s]')
legend('State Trajectory', 'Desired State', 'Transition')