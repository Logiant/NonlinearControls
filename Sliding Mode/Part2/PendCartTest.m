clear; close all; clc;
syms g m M L F theta theta_dot x x_dot

A = [m*L*cos(theta) m*L^2; ...
    m+M,     m*L*cos(theta)];

B = [g*m*L*sin(theta); m*L*theta_dot^2*sin(theta)+F];

R = inv(A)*B

%% plotting simulink results
t = ScopeData(:,1); th_d = ScopeData(:,2);
PID = ScopeData(:,3); SMC = ScopeData(:,4);

plot(t, th_d, '--k', ...
     t, PID, '-b', ...
     t, SMC, '-g')

 xlabel('Time [s]');
 ylabel('\theta [radians]')
 legend('Desired Position', 'PID Response', 'SMC Response');