function [Dx,Dz,D] = DragModelPlanar(vx,vz,p)

V = sqrt(vx^2+vz^2);

if V == 0
    Dx = 0;
    Dz = 0;
    D = 0;
else
    D = 0.5*p.rho*V^2*p.Cd*p.Aref;
    Dx = -D*(vx/V);
    Dz = -D*(vz/V);
end

end