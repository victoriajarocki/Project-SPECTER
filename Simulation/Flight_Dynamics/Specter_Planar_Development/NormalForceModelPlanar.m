function [Nx,Nz,N,CN,alpha] = NormalForceModelPlanar(z,vx,vz,theta,p)

V = sqrt(vx^2 + vz^2);

if V == 0
    Nx = 0;
    Nz = 0;
    N = 0;
    CN = 0;
    alpha = 0;
    return
end

gamma = atan2(vx,vz);

alpha = theta - gamma;

[~,~,rho,~] = AtmosphereModelPlanar(z,p);

q = 0.5*rho*V^2;

CN = p.CNa*alpha;

N = q*p.Aref*CN;

Nx = N*(vz/V);
Nz = -N*(vx/V);

end