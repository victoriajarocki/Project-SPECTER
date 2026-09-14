function [Ta,P,rho,a] = AtmosphereModel(h,p)

if h<0
    h=0;
end

if h>11000
    error('AtmosphereModel currently supports altitudes up to 11 km.')
end

Ta = p.T0_atm -p.L*h;

P = p.P0*(Ta/p.T0_atm)^(p.g/(p.R*p.L));

rho = P/(p.R*Ta);

a = sqrt(p.gamma*p.R*Ta);

end