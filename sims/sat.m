function y = sat(x, U)

ind = abs(x) < U;
y = x/U .* ind;
y = y + sign(x) .* (1-ind);

end