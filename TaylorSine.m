clear; clc;
theta = -pi/2:0.01:pi/2;

s = sin(theta);
l = theta;

figure;
% axis for degrees
b=axes('Position',[.1 .1 .8 1e-12]);
set(b,'Units','normalized');
set(b,'Color','none');

% axis for radians
a=axes('Position',[.1 .2 .8 .7]);
set(a,'Units','normalized');
plot(a,theta,s, theta, l, '--');

% set limits and labels
set(a,'xlim',[theta(1) theta(length(theta))]);
set(b,'xlim',[theta(1)*180/pi theta(length(theta))*180/pi]);
xlabel(a,'\theta [radians]')
xlabel(b,'\theta [degrees]')
ylabel(a,'sin(\theta)');
grid on

legend('True Value', 'Linear Approximation')