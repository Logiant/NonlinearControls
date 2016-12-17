clear; close all; clc;

g = 9.81; %m/s^2
L = 9.81; %m
b = 1.5; %damping
m = 1; %kg

thetaDot = @(theta, omega) omega;
omegaDot = @(theta, omega) -g/L*sin(theta) - b*omega;
energy = @(theta, omega) m*g*L*(1-cos(theta)) + 1/2*m*L^2*omega.^2;
energyDot = @(theta, omega) -b*m*L^2*omega.^2;

[th, om] = meshgrid(-1/2*pi:.2:1/2*pi, -1:.1:1);

u = thetaDot(th, om); v = omegaDot(th, om);
L = sqrt(u.^2 + v.^2);

figure(1);
quiver(th, om, u./L, v./L, 0.5)
hold on
thi = pi/2; omi = -0.5;
t = 0:0.000000001:0.00000008;
for i = 1:length(t)-1
    thi(i+1) = thi(i) + thetaDot(thi(i), omi(i));
    omi(i+1) = omi(i) + omegaDot(thi(i), omi(i));
end
plot(thi, omi, '.-k')

hold off


figure(2)
E = energy(th, om);
surf(th, om, E)
hold on
%plot3(thi, omi, energy(thi, omi), '-k');
hold off;
xlabel('\theta');
ylabel('\omega');
zlabel('Energy');

figure(3)
Pout = -energyDot(th, om);
surf(th, om, Pout)
xlabel('\theta');
ylabel('\omega');
zlabel('Power Dissapated');