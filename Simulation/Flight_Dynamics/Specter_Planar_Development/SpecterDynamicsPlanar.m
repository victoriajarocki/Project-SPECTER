function dX = SpecterDynamicsPlanar(t,X,p)

x     = X(1);
z     = X(2);
vx    = X(3);
vz    = X(4);
theta = X(5);
omega = X(6);
delta = X(7);

% Parameters
g = p.g;
l = p.l;
I = p.I;

% Commanded gimbal angle
T = ThrustModelPlanar(t,p);

if T > 0
    deltaCmd = AttitudeControllerPlanar(theta,omega,p);
else
    deltaCmd = 0;
end
% Servo dynamics
deltaDotRaw = (deltaCmd - delta)/p.tauServo;

% Servo rate limit
deltaDot = max(min(deltaDotRaw,p.deltaRateMax), ...
               -p.deltaRateMax);

% Vehicle models
m = MassModelPlanar(t,p);

[Dx,Dz,~] = DragModelPlanar(z,vx,vz,p);

[Nx,Nz,~,~,~] = ...
    NormalForceModelPlanar(z,vx,vz,theta,p);

Maero = AeroMomentModelPlanar(theta,Nx,Nz,p);

% Translational dynamics
ax = (T*sin(theta + delta) + Dx + Nx)/m;
az = (T*cos(theta + delta) + Dz + Nz)/m - g;

% Rotational dynamics
Mtvc = l*T*sin(delta);

thetaDDot = (Mtvc + Maero)/I;



T = ThrustModelPlanar(t,p);


% State derivatives
dX = [vx;
      vz;
      ax;
      az;
      omega;
      thetaDDot;
      deltaDot];

end