clear; close all; clc;

R = 500;
L = 0.001;
C = 0.001;

eCDot = @(eC, eCDot) eCDot;
eCDotDot = @(eC, eCDot) -eC/(L*C) - R/L*eCDot;
energy = @(eC, eCDot) C/2*(eC).^2 + L/2*(C*eCDot).^2;
energyDot = @(eC, eCDot) -C^2*R*eCDot.^2;

[ec, ecD] = meshgrid(-1:.2:1, -1000:100:1000);

u = eCDot(ec, ecD); v = eCDotDot(ec, ecD);
L = sqrt(u.^2 + v.^2);

figure(1);
quiver(ec, ecD, u./L, v./L, 0.5)
hold on
eci = pi/2; ecDi = -0.5;
t = 0:0.000000001:0.00000008;
for i = 1:length(t)-1
    eci(i+1) = eci(i) + eCDot(eci(i), ecDi(i));
    ecDi(i+1) = ecDi(i) + eCDotDot(eci(i), ecDi(i));
end
plot(eci, ecDi, '.-k')

hold off


figure(2)
E = energy(ec, ecD);
surf(ec, ecD, E)
hold on
%plot3(eci, ecDi, energy(eci, ecDi), '-k');
hold off;
xlabel('eC');
ylabel('eCDot');
zlabel('Energy');

figure(3)
Pout = -energyDot(ec, ecD);
surf(ec, ecD, Pout)
xlabel('eC');
ylabel('eCDot');
zlabel('Power Dissapated');