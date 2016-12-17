clear; close all; clc;

k = 1;
b = 1.5; %damping
m = 1; %kg
Cd = 1;

xDot = @(x, v) v;
vDot = @(x, v) -Cd/m*v.^2 - k/m*x;
energy = @(x, v) m/2*v.^2 + k/2*x.^2;
energyDot = @(x, v) -Cd*v.^2;

[xi, vi] = meshgrid(-pi/2:.2:pi/2, -1:.1:1);

u = xDot(xi, vi); v = vDot(xi, vi);
L = sqrt(u.^2 + v.^2);

figure(1);
quiver(xi, vi, u./L, v./L, 0.5)
hold on
xii = pi/2; vii = -0.5;
t = 0:0.000000001:0.00000008;
for i = 1:length(t)-1
    xii(i+1) = xii(i) + xDot(xii(i), vii(i));
    vii(i+1) = vii(i) + vDot(xii(i), vii(i));
end
plot(xii, vii, '.-k')

hold off


figure(2)
E = energy(xi, vi);
surf(xi, vi, E)
hold on
%plot3(xii, vii, energy(xii, vii), '-k');
hold off;
xlabel('x');
ylabel('v');
zlabel('Energy');

figure(3)
Pout = -energyDot(xi, vi);
surf(xi, vi, Pout)
xlabel('x');
ylabel('v');
zlabel('Power Dissapated');