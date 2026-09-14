function dx = SpecterDynamics(t,x,p)

h = x(1);
v = x(2);

% Models
T = ThrustModel(t,p);
m = MassModel(t,p);
[~,~,rho,~] = AtmosphereModel(h,p);

D = 0.5*rho*p.Cd*p.A*v*abs(v);

dh = v;
dv = (T-D-m*p.g)/m;
dx = [dh;dv];

end