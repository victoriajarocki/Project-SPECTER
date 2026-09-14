function deltaCmd = AttitudeControllerPlanar(theta,omega,p)

thetaError = p.thetaCmd - theta;

deltaCmd = p.Kp*thetaError - p.Kd*omega;

deltaCmd = max(min(deltaCmd,p.deltaMax), ...
               -p.deltaMax);

end