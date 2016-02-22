function ret = CircuitPlant(ecI, ecDotI, switchVal)

    global R L C eSrc;

    if switchVal > 0
        switchVal = 1;
    else 
        switchVal = 0;
    end
    
    ecDotDot = (eSrc*switchVal - ecDotI*C*R - ecI)/(L*C);
    ecDot = ecDotI;
    
    ret = [ecDot, ecDotDot];
end

