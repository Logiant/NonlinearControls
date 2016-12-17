function y = sat(x, U)

y = sign(x);
if (abs(x) < U)    
    y = U * x;
end

end