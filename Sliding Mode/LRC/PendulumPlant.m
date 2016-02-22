function ret = PendulumPlant(thetaI, thetaDotI, torque)

global G L

thetaDot = thetaDotI;
thetaDotDot = -G/L*sin(thetaI) + torque;

ret = [thetaDot, thetaDotDot];

end

