clear; close all; clc;

g = 9.81;
L = 9.81;
b = 0;

tau = @(x1, x2) g/L*(x1.^2 - pi^2)*0;
x1dot = @(x1, x2) x2;
x2dot = @(x1, x2) tau(x1, x2) - g/L*sin(x1) - b*x2;


[x1, x2] = meshgrid(-2*pi:0.5:4*pi, -12:0.5:12);

x1d = x1dot(x1, x2);
x2d = x2dot(x1, x2);

L = sqrt(x1d.^2 + x2d.^2);

quiver(x1, x2, x1d./L, x2d./L);
xlabel('x1'); ylabel('x2');
axis equal