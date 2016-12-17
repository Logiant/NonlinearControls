clear; close all; clc;

%controller variables
lambda = 1.75;  % s - 1/slope of sliding surface
beta = 7;       % N-M - Torque Magnitude
%solution variables
tf = 20;
thD = 0.5;    % rad - desired angle
%system parameters
g = 9.81;   % m/s2 - gravity
L = 1;      % m - pendulum length
m = 0.25;   % kg - mass of end
b = 0.5;    %     damping coefficient
%system equations
x1dot = @(x1, x2) x2;
x2dot = @(x1, x2, T) -g/L*sin(x1) -b/(m*L^2)*x2 + 1/(m*L^2)*T;
%% phase plane plot solution space 1
[x1, x2] = meshgrid(-5:0.25:5, -5:0.25:5);

Tin = beta;
x1d = x1dot(x1, x2) .* ( ((thD-x1) + lambda*(-x2)) > 0 );
x2d = x2dot(x1, x2, Tin) .* ( ((thD-x1) + lambda*(-x2)) > 0 );

size = sqrt(x1d.^2 + x2d.^2);
quiver(x1, x2, x1d./size, x2d./size, 'c');

% phase plane plot solution space 2
clear x1d x2d;

Tin = -beta;
x1d = x1dot(x1, x2) .* ( ((thD-x1) + lambda*(-x2)) < 0 );
x2d = x2dot(x1, x2, Tin) .* ( ((thD+-x1) + lambda*(-x2)) < 0 );
size = sqrt(x1d.^2 + x2d.^2);
hold on
quiver(x1, x2, x1d./size, x2d./size, 'r');
%plot axis setup
axis square
xlabel('Theta');
ylabel('Theta Dot');


%% plot vs time
th0 = -2; thDot0 = -3;

s = @(th, thDot) sign(((lambda)*(-thDot) + (thD - th)))*beta;

dt = 0.01;
t = 0:dt:tf;

th(1) = th0;
thDot(1) = thDot0;

for i = 2:length(t)
    K1 = x1dot(th(i-1), thDot(i-1))*dt;
    L1 = x2dot(th(i-1), thDot(i-1), s(th(i-1), thDot(i-1)))*dt;
    
    K2 = x1dot(th(i-1) + K1/2, thDot(i-1) + L1/2)*dt;
    L2 = x2dot(th(i-1) + K1/2, thDot(i-1) + L1/2, s(th(i-1), thDot(i-1)))*dt;
    
    K3 = x1dot(th(i-1) + K2/2, thDot(i-1) + L2/2)*dt;
    L3 = x2dot(th(i-1) + K1/2, thDot(i-1) + L1/2, s(th(i-1), thDot(i-1)))*dt;
    
    K4 = x1dot(th(i-1) + K3  , thDot(i-1) + L3)*dt;
    L4 = x2dot(th(i-1) + K1/2, thDot(i-1) + L1/2, s(th(i-1), thDot(i-1)))*dt;
    
    th(i) = th(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    thDot(i) = thDot(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;
    
end

plot(th, thDot, '.-k')


%% phase plane plot solution space 1 - ERROR SPACE
x1dotE = @(x1, x2) x2;
x2dotE = @(x1, x2, T) g/L*sin(thD - x1) - 1/(m*L^2)*T;

[x1, x2] = meshgrid(-5:0.25:5, -5:0.25:5);

clear x1d x2d;
Tin = beta;
x1d = x1dotE(x1, x2) .* ( (lambda*x2 + x1) > 0 );
x2d = x2dotE(x1, x2, Tin) .* ( (lambda*x2 + x1) > 0 );

size = sqrt(x1d.^2 + x2d.^2);
figure(2);
quiver(x1, x2, x1d./size, x2d./size, 'c');

% phase plane plot solution space 2 - ERROR SPACE
clear x1d x2d;

Tin = -beta;
x1d = x1dotE(x1, x2) .* ( (lambda*x2 + x1) < 0 );
x2d = x2dotE(x1, x2, Tin) .* ( (lambda*x2 + x1) < 0 );
size = sqrt(x1d.^2 + x2d.^2);
hold on
quiver(x1, x2, x1d./size, x2d./size, 'r');
%plot axis setup
axis square
xlabel('Theta Error');
ylabel('Theta Dot Error');

