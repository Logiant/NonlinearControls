%Initialize
clear; close all; clc;
%simulation & controller input
lambda = 1;

%system parameters
L = 0.001;     %H
C = 1/L; %F
eIn = 3.3;
%solution parameters voltage
eD = eIn/2;
tf = 10;
dt = 0.001;
%system equations
x1dot = @(x1, x2) x2;
x2dot = @(x1, x2, ea) -1/(L*C)*(x1 - ea);
%% PLOT 1 - System Space
[x1, x2] = meshgrid(-5:0.25:5, -5:0.25:5);
%Switch is closed (ea = eIn)
B = eIn;
%half of system equations
x1d = x1dot(x1, x2) .* ( (lambda*(-x2) + (eD-x1)) > 0 );
x2d = x2dot(x1, x2, B) .* ( (lambda*(-x2) + (eD-x1)) > 0 );
%plot system
size = sqrt(x1d.^2 + x2d.^2);
quiver(x1, x2, x1d./size, x2d./size, 'c');
%lower half of system space
clear x1d x2d;
%switch is open (ea = 0)
B = 0;
%half of the system space
x1d = x1dot(x1, x2) .* ( (lambda*(-x2) + (eD-x1)) < 0 );
x2d = x2dot(x1, x2, B) .* ( (lambda*(-x2) + (eD-x1)) < 0 );
size = sqrt(x1d.^2 + x2d.^2);
%plot 2nd half
hold on
quiver(x1, x2, x1d./size, x2d./size, 'r');
axis square
xlabel('eC');
ylabel('eC-Dot');
title('System Response');
%% plot vs time
ec0 = 0; ecDot0 = 0;


s = @(ec, ecDot) ( (lambda*(-ecDot) + (eD-ec)) >= 0 )*eIn;

t = 0:dt:tf;

ec(1) = ec0;
ecDot(1) = ecDot0;

for i = 2:length(t)
    K1 = x1dot(ec(i-1), ecDot(i-1))*dt;
    L1 = x2dot(ec(i-1), ecDot(i-1), s(ec(i-1), ecDot(i-1)))*dt;
    
    K2 = x1dot(ec(i-1) + K1/2, ecDot(i-1) + L1/2)*dt;
    L2 = x2dot(ec(i-1) + K1/2, ecDot(i-1) + L1/2, s(ec(i-1), ecDot(i-1)))*dt;
    
    K3 = x1dot(ec(i-1) + K2/2, ecDot(i-1) + L2/2)*dt;
    L3 = x2dot(ec(i-1) + K1/2, ecDot(i-1) + L1/2, s(ec(i-1), ecDot(i-1)))*dt;
    
    K4 = x1dot(ec(i-1) + K3  , ecDot(i-1) + L3)*dt;
    L4 = x2dot(ec(i-1) + K1/2, ecDot(i-1) + L1/2, s(ec(i-1), ecDot(i-1)))*dt;
    
    ec(i) = ec(i-1) + 1/6*K1 + 1/3*K2 + 1/3*K3 + 1/6*K4;
    ecDot(i) = ecDot(i-1) + 1/6*L1 + 1/3*L2 + 1/3*L3 + 1/6*L4;
    
end

plot(ec, ecDot, '.-k')



%% ERROR SPACE PLOT
clear x1d x2d;

x1dotE = @(x1, x2) x2;
x2dotE = @(x1, x2, ea) 1/(L*C)*(eD - x1 - ea);


[x1, x2] = meshgrid(-5:0.25:5, -5:0.25:5);

B = eIn;
x1d = x1dotE(x1, x2) .* (x1 + lambda*x2 > 0);
x2d = x2dotE(x1, x2, B) .* (x1 + lambda*x2 > 0);

size = sqrt(x1d.^2 + x2d.^2);
figure();
quiver(x1, x2, x1d./size, x2d./size, 'c');

clear x1d x2d;
B = 0;
x1d = x1dotE(x1, x2) .* (x1 + lambda*x2 < 0);
x2d = x2dotE(x1, x2, B) .* (x1 + lambda*x2 < 0);
size = sqrt(x1d.^2 + x2d.^2);
hold on
quiver(x1, x2, x1d./size, x2d./size, 'r');
axis square
xlabel('eCE');
ylabel('eCE-Dot');
title('System Response in Error Space');