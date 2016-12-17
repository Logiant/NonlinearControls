clear; close all; clc;

%controller variables
lambda = 1;  % s - 1/slope of sliding surface
beta = 5;       % N-M - Torque Magnitude
%solution variables
tf = 20;
xD = 2;    % m - desired position
%system parameters
Cd = 3;   % m/s2 - gravity
m = 5;      % m - pendulum lengx
b = 2.25;   % kg - mass of end
k = 1.5;    %     damping coefficient
%system equations
x1dot = @(x1, x2) x2;
x2dot = @(x1, x2, F) -Cd/m*(x2.*abs(x2)) - k/m*x1 + F/m;
%% phase plane plot solution space 1
[x1, x2] = meshgrid(-5:0.25:5, -5:0.25:5);

Tin = beta;
x1d = x1dot(x1, x2) .* ( ((xD-x1) + lambda*(-x2)) > 0 );
x2d = x2dot(x1, x2, Tin) .* ( ((xD-x1) + lambda*(-x2)) > 0 );

size = sqrt(x1d.^2 + x2d.^2);
quiver(x1, x2, x1d./size, x2d./size, 'c');

% phase plane plot solution space 2
clear x1d x2d;

Tin = -beta;
x1d = x1dot(x1, x2) .* ( ((xD-x1) + (lambda*(-x2))) < 0 );
x2d = x2dot(x1, x2, Tin) .* ( ((xD+-x1) + (lambda*(-x2))) < 0 );
size = sqrt(x1d.^2 + x2d.^2);
hold on
quiver(x1, x2, x1d./size, x2d./size, 'r');
%plot axis setup
axis square
xlabel('x [m]');
ylabel('v [m/s]');


%% plot vs time
x0 = -2; xDot0 = -1;

s = @(x, xDot) sign(((lambda)*(-xDot) + (xD - x)))*beta;

dt = 0.01;
t = 0:dt:tf;

x(1) = x0;
xDot(1) = xDot0;

for i = 2:length(t)
    K1 = x1dot(x(i-1), xDot(i-1))*dt;
    L1 = x2dot(x(i-1), xDot(i-1), s( x(i-1), xDot(i-1) ))*dt;
    
    K2 = x1dot(x(i-1) + K1/2, xDot(i-1) + L1/2)*dt;
    L2 = x2dot(x(i-1) + K1/2, xDot(i-1) + L1/2, s(x(i-1), xDot(i-1)))*dt;
    
    K3 = x1dot(x(i-1) + K2/2, xDot(i-1) + L2/2)*dt;
    L3 = x2dot(x(i-1) + K1/2, xDot(i-1) + L1/2, s(x(i-1), xDot(i-1)))*dt;
    
    K4 = x1dot(x(i-1) + K3  , xDot(i-1) + L3)*dt;
    L4 = x2dot(x(i-1) + K1/2, xDot(i-1) + L1/2, s(x(i-1), xDot(i-1)))*dt;
    
    x(i) = x(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    xDot(i) = xDot(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;
    
end

plot(x, xDot, '.-k')


%% phase plane plot solution space 1 - ERROR SPACE
% x1dotE = @(x1, x2) x2;
% x2dotE = @(x1, x2, T) g/L*sin(xD - x1) - 1/(m*L^2)*T;
% 
% [x1, x2] = meshgrid(-5:0.25:5, -5:0.25:5);
% 
% clear x1d x2d;
% Tin = beta;
% x1d = x1dotE(x1, x2) .* ( (lambda*x2 + x1) > 0 );
% x2d = x2dotE(x1, x2, Tin) .* ( (lambda*x2 + x1) > 0 );
% 
% size = sqrt(x1d.^2 + x2d.^2);
% figure(2);
% quiver(x1, x2, x1d./size, x2d./size, 'c');
% 
% % phase plane plot solution space 2 - ERROR SPACE
% clear x1d x2d;
% 
% Tin = -beta;
% x1d = x1dotE(x1, x2) .* ( (lambda*x2 + x1) < 0 );
% x2d = x2dotE(x1, x2, Tin) .* ( (lambda*x2 + x1) < 0 );
% size = sqrt(x1d.^2 + x2d.^2);
% hold on
% quiver(x1, x2, x1d./size, x2d./size, 'r');
% %plot axis setup
% axis square
% xlabel('xeta Error');
% ylabel('xeta Dot Error');
% 
