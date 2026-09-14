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
[Dx,Dz,D] = DragModelPlanar(vx,vz,p);

ax = (T*sin(theta+delta)+Dx)/m;
az = (T*cos(theta+delta)+Dz)/m-g;

alpha = (l*T*sin(delta))/I;

dX = [vx;
    vz;
    ax;
    az;
    omega;
    alpha];

end