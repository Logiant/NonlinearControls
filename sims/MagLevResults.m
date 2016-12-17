close all;
figure(1);
hold on
plot(ResponsePID(:,1), ResponsePID(:,2), '--', ResponsePID(:,1), ResponsePID(:, 3), 'r')
hold on
plot(ResponseSMC(:,1), ResponseSMC(:, 3), '.-k','markersize', 10)
legend('Desired', 'PID', 'SMC')
axis([0 2 0 1]);
xlabel('time [s]'); ylabel('Fraction of Total Height');
box on;