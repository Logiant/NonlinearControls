clear; close all; clc;
global m g L b
m = 15; %kg
g = 9.81; %m/s/s
L = 1.7; %m
b = 20; %N-m-s

tau = @(x1, x2) g/L*(x1.^2 - pi^2)*0;
x1dot = @(x1, x2) x2;
x2dot = @(x1, x2) tau(x1, x2) - g/L*sin(x1) - b/(m*L^2)*x2;


[x1, x2] = meshgrid(-2*pi:0.5:4*pi, -5:0.5:5);

x1d = x1dot(x1, x2);
x2d = x2dot(x1, x2);


quiver(x1, x2, x1d, x2d, 2, 'Color', [.65 .85 .65]);
xlabel('\theta [radians]'); ylabel('\omega [radians/second]');
axis equal tight;
hold on

%% solve some dynamics problems!
for x0 = [3.5 8];
w0 = -x0+3.5;
tf = 100; %second
[t, y] = ode45(@pendulumplantquick, [0 tf], [x0 w0]);
plot(y(:,1), y(:,2), 'linewidth', 3);
end

for w0 = 4;
x0 = 0;
tf = 100; %second
[t, y] = ode45(@pendulumplantquick, [0 tf], [x0 w0]);
plot(y(:,1), y(:,2), 'linewidth', 3);
end

tf = 100; %second
[t, y] = ode45(@pendulumplantquick, [0 tf], [-4 5]);
plot(y(:,1), y(:,2), 'linewidth', 3)

legend('Portrait', '(3.5, 0)', '(8,-4.5)', '(0,4)', '(-4,5)');


