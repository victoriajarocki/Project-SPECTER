function dX = SpecterDynamicsPlanar(t,X,p)

x     = X(1);
z     = X(2);
vx    = X(3);
vz    = X(4);
theta = X(5);
omega = X(6);

% Parameters
g     = p.g;
l     = p.l;
I     = p.I;
delta = p.delta;

m     = MassModelPlanar(t,p);
T     = ThrustModelPlanar(t,p);

ax = (T/m)*sin(theta+delta);
az = (T/m)*cos(theta+delta)-g;

alpha = (l*T*sin(delta))/I;

dX = [vx;
    vz;
    ax;
    az;
    omega;
    alpha];

end