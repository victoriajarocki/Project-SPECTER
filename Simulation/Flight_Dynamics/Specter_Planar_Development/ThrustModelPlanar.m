function T = ThrustModelPlanar(t,p)

if t <= p.tb
    T = interp1(p.thrustTime,p.thrustData,t,'linear');
else
    T = 0;
end

end
