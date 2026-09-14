clear
clc
close all

%% Parameters

% Vehicle
p.l = 0.3;
p.I = 0.5;

% Mass
p.mWet = 10;
p.mDry = 9;

% Atmosphere
p.T0 = 288.15;
p.P0 = 101325;
p.L = 0.0065;
p.R = 287.05;
p.gammaAir = 1.4;
p.g = 9.80665;

% Aerodynamics
p.Cd = 0.5;
p.Aref = 0.01;
p.CNa = 2.0;

% Stability
p.dCP = 0.15;

% Thrust
p.thrustTime = [0 0.1 0.3 1.0 1.8 2.0];
p.thrustData = [0 150 220 210 180 0];
p.tb = p.thrustTime(end);

% Gimbal / Servo
p.deltaMax = deg2rad(5);
p.deltaRateMax = deg2rad(60);
p.tauServo = 0.05;

% Attitude Command
p.thetaCmd = deg2rad(0);

%% Initial Conditions

x0 = 0;
z0 = 0;

vx0 = 0;
vz0 = 0;

theta0 = deg2rad(2);
omega0 = 0;

delta0 = 0;

X0 = [x0;
      z0;
      vx0;
      vz0;
      theta0;
      omega0;
      delta0];

%% Liftoff Calculation

liftoffBalance = @(t) ...
    ThrustModelPlanar(t,p) - ...
    MassModelPlanar(t,p)*p.g;

tLiftoff = fzero( ...
    liftoffBalance, ...
    [0 0.1]);

%% Simulation Settings

tfinal = 2;

solverOptions = odeset( ...
    'RelTol',1e-6, ...
    'AbsTol',1e-8);

%% Gain Sweep Values

KpValues = [1.0 1.5 2.0 2.5 3.0];

KdValues = [0.25 0.5 0.75 1.0 1.25];

%% Results Storage

results = [];

%% Controller Gain Sweep

for i = 1:length(KpValues)

    for j = 1:length(KdValues)

        p.Kp = KpValues(i);
        p.Kd = KdValues(j);

        dynamics = @(t,X) ...
            SpecterDynamicsPlanar(t,X,p);

        [t,X] = ode45( ...
            dynamics, ...
            [tLiftoff tfinal], ...
            X0, ...
            solverOptions);

        %% State Histories

        theta = X(:,5);
        omega = X(:,6);
        delta = X(:,7);

        %% Commanded Gimbal History

        deltaCmdHist = zeros(size(t));

        for k = 1:length(t)

            deltaCmdHist(k) = ...
                AttitudeControllerPlanar( ...
                    theta(k), ...
                    omega(k), ...
                    p);

        end

        %% Controller Metrics

        thetaDeg = rad2deg(theta);
        thetaCmdDeg = rad2deg(p.thetaCmd);

        finalError = ...
            abs(thetaDeg(end) - thetaCmdDeg);

        maxPitchRate = ...
            max(abs(rad2deg(omega)));

        maxCommandedGimbal = ...
            max(abs(rad2deg(deltaCmdHist)));

        maxActualGimbal = ...
            max(abs(rad2deg(delta)));

        %% Settling Time

        initialErrorDeg = ...
            abs(thetaDeg(1) - thetaCmdDeg);

        settlingBand = ...
            0.02*initialErrorDeg;

        settlingTime = NaN;

        for k = 1:length(t)

            remainingError = ...
                abs(thetaDeg(k:end) - ...
                    thetaCmdDeg);

            if all(remainingError <= settlingBand)

                settlingTime = t(k);

                break

            end

        end

        %% Store Results

        results = [results;
            p.Kp, ...
            p.Kd, ...
            finalError, ...
            maxPitchRate, ...
            maxCommandedGimbal, ...
            maxActualGimbal, ...
            settlingTime];

    end

end

%% Results Table

ResultsTable = array2table( ...
    results, ...
    'VariableNames', ...
    {'Kp', ...
     'Kd', ...
     'FinalError_deg', ...
     'MaxPitchRate_deg_s', ...
     'MaxCmdGimbal_deg', ...
     'MaxActualGimbal_deg', ...
     'SettlingTime_s'});

fprintf('\n--- ALL CONTROLLER RESULTS ---\n\n');

disp(ResultsTable)

%% Sorted Results

SortedResults = sortrows( ...
    ResultsTable, ...
    'SettlingTime_s');

fprintf('\n--- RESULTS SORTED BY SETTLING TIME ---\n\n');

disp(SortedResults)