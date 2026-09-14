function Maero = AeroMomentModelPlanar(theta,Nx,Nz,p)

rx = -p.dCP*sin(theta);
rz = -p.dCP*cos(theta);

Maero = rz*Nx - rx*Nz;

end