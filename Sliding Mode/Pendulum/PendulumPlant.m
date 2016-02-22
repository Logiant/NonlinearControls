function ret = PendulumPlant(thetaI, thetaDotI, torque)

global G L

theta = thetaDotI;
thetaDot = -G/L*sin(thetaI) + torque;

ret = [theta, thetaDot];

end

