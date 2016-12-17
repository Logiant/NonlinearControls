%rocket maaaan
clear; close all; clc;
%controller parameters
beta = 5; lambda = 1;
thresh = 5.5; %threshold for regime change
%rocket parameters
roe = 1.225; % kg/m3 - air density
Cd = 0.75; %drag coeff.
A = 0.0021;% m2 - cross sectional area
g0 = 9.8; % m/s2 - gravity of earth
g = g0;
Isp = 100; % s - specific impulse
%dynamic equations
x_dot = @(x, v, m, mdot) v;
v_dot = @(x, v, m, mdot) -(1/2*roe*Cd*A/m)*v.^2 - g + (g0/m*Isp-v/m).*mdot;
% Kinematic parameters
x_d = 1200; %m
x0 = 0; v0 = 0;
%controller
s = @(x, v) lambda*(x-x_d) + v;
u_eq = @(x,v,m) (1/2*roe*Cd*A/m.*v.*abs(v) + g - lambda*v)./(g0/m*Isp - v./m);
u = @(x, v, m) min(u_eq(x, v, m),0) + max(-beta*sign(s(x,v)), 0);
%temporal parameters
tf = 10; dt = 0.01; t = 0:dt:tf;
%initial conditions
i = 1;
x(i) = x0;
v(i) = v0;
m = 5; % kg - dry mass
mf(i) = 7.5; %kg - fuel
mdot(i) = 0;
i_cross = 0;
for i = 2:length(t)

    x(i) = x(i-1) + x_dot(x(i-1), v(i-1), mf(i-1)+m, mdot(i-1))*dt;
    x(i) = max(x(i), 0); %don't penetrate the ground
    v(i) = v(i-1) + v_dot(x(i-1), v(i-1), mf(i-1)+m, mdot(i-1))*dt;
    v(i) = max(v(i), -x(i)/dt);  %don't drop below ground
    mdot(i) = u(x(i-1), v(i-1), mf(i-1)+m);
    mdot(i) = min(mf(i-1)/dt, mdot(i)); %don't go below 0 fuel
    mf(i) = mf(i-1) - mdot(i-1)*dt;
    if abs(s(x(i-1), v(i-1))) < thresh && i_cross == 0
        i_cross = i-1;
    end
end

plot(t, x, [0 tf], [x_d, x_d], t(i_cross), x(i_cross), 'or')
xlabel('Time [s]'); ylabel('Altitude [m]');
legend('Response', 'Desired Position', 'Transition')
figure
plot(x, v, x(i_cross), v(i_cross), 'or')
xlabel('Altitude [m]'); ylabel('Velocity [m/s]')
legend('State Trajectory', 'Transition')