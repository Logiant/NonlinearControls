clear; close all; clc;

lambda = -1;
tf = 20;

L=0.1;
C=1/L;

eIn = 1;
eD = eIn/2;


x1dot = @(x1, x2) x2;
x2dot = @(x1, x2, ea) 1/(L*C)*(ea - x1);

[x1, x2] = meshgrid(-5:0.25:5, -5:0.25:5);

B = eIn;
x1d = x1dot(x1, x2) .* (((-lambda*x1)+x2)<0+eD);
x2d = x2dot(x1, x2, B) .* (((-lambda*x1)+x2)<0+eD);

size = sqrt(x1d.^2 + x2d.^2);
quiver(x1, x2, x1d./size, x2d./size, 'c');

%%
clear x1d x2d;

B = 0;
x1d = x1dot(x1, x2) .* (((-lambda*x1)+x2)>0+eD);
x2d = x2dot(x1, x2, B) .* (((-lambda*x1)+x2)>0+eD);
size = sqrt(x1d.^2 + x2d.^2);
hold on
quiver(x1, x2, x1d./size, x2d./size, 'r');
axis square
xlabel('eC');
ylabel('eC-Dot');

%% plot vs time
ec0 = 3; ecDot0 = 1.5;


s = @(ec, ecDot) (((-lambda)*ec + ecDot) <= eD)*eIn;

dt = 0.01;
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



