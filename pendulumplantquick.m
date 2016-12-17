function dydt = pendulumplantquick(t, y)
global m g L b

y1dot =  y(2);
y2dot = - g/L*sin(y(1)) - b/(m*L^2)*y(2);

dydt = zeros(2, 1);

dydt(1) = y1dot;
dydt(2) = y2dot;
end