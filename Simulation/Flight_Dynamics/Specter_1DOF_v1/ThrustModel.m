function T = ThrustModel(t,p)

if t>=p.thrustTime(1) && t<= p.thrustTime(end)
    T = interp1(p.thrustTime,p.thrustData,t,'linear');
else
    T = 0;

end

end
