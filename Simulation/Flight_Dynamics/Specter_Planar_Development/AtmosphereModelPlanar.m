function [Tair,P,rho,a] = AtmosphereModelPlanar(z,p)

h = max(z,0);

Tair = p.T0 - p.L*h;

P = p.P0*(Tair/p.T0)^(p.g/(p.R*p.L));

rho = P/(p.R*Tair);

a = sqrt(p.gammaAir*p.R*Tair);

end