clear
% TODO: create a class or function to avoid a huge main code.
% You could have:
% - One satellite class, that has methods to simulate and then play in the
% satellite scenario + post processing methods.
% - A plotting class.

%% Earth constants
Re = earthRadius; % [m]
mi = 398600.418e9; % [m^3/s^2]

%% Orbital parameters and initial conditions
a = Re + 3000e3; % [m]
e = 0.3;
inclination = 0; %[deg]
raan = 0; % Right Ascension of Ascending node [deg]
aop = 0; % Arguement of Pericenter [deg]
ta = 0; % True amomaly [deg]
T = 2 * pi * sqrt(a^3 / mi); % [s]

% Define 3 rotation angles wrt to ECI frame
theta3 = deg2rad(0);
theta2 = deg2rad(0);
theta1 = deg2rad(0);

q0 = angle2quat(theta3,theta2,theta1,"ZYX");
w0 = [0,0,0]; % Angular velocity in body frame [deg/s]
startTime = datetime(2020,1,1,12,0,0);
startTimeJD = juliandate(startTime);
simLength = T;

%% Setup simulation
model = "simulink/satellite_propagator.slx";
simIn = Simulink.SimulationInput(model);
simIn = simIn.setModelParameter("StopTime", num2str(simLength), ...
    "Solver","ode4", ...
    "FixedStep","1");

%% Run simulation
simOut = sim(simIn);
t = simOut.tout;

%% Do some post processing
% Timeseries
Rsat_ts = simOut.yout{1}.Values;
Qin2body_ts = simOut.yout{4}.Values;

% Vectors
Rsat = Rsat_ts.Data;
Qin2body = Qin2body_ts.Data;

% Find the timetsamps in UTC
t_utc = startTime + seconds(t);

% Inverse quaternion to go from body to inertial
Qbody2in = quatinv(Qin2body);

% Find direction of line of sight, considered as exiting from the x axis of
% the satellite.
LOS_hat = quatrotate(Qbody2in,[-1,0,0]);

% Intersection between line of sight and earth surface
rho = -dot(LOS_hat,Rsat,2) - sqrt((dot(LOS_hat,Rsat,2)).^2 - vecnorm(Rsat,2,2).^2 + Re^2);
rho(imag(rho) ~= 0) = 0;
rho(rho<0) = 0;
indexes = rho ~= 0;

% Find the ground track vector
Rtar = rho.*LOS_hat;
Rgt = Rsat + Rtar;
Rgt(~indexes,:) = 0;

% Find ground track of satellite
Rsat_gt = Rsat .* (Re ./ vecnorm(Rsat, 2, 2));

% Convert to ECEF
Rsat_ecef = helpers.eci2ecef(t_utc,Rsat);
Rgt_ecef = helpers.eci2ecef(t_utc,Rgt);

% Latitude and longitude
lla_sat = ecef2lla(Rsat_ecef,0,Re);
ll_sat = lla_sat(:,1:2);
lla_gt = ecef2lla(Rgt_ecef,0,Re);
ll_gt = lla_gt(:,1:2);

% Plot ground tracks
figure(1)
geoplot(ll_sat(:,1),ll_sat(:,2))
hold on
geoplot(ll_gt(:,1),ll_gt(:,2))
legend("Satellite","LoS")
geobasemap("satellite")

% Quick plot of the orbit to check
figure(2)
plot3(Rsat(:,1),Rsat(:,2),Rsat(:,3))
hold on
plot3(Rsat_gt(:,1), Rsat_gt(:,2), Rsat_gt(:,3))
plot3(Rgt(:,1),Rgt(:,2),Rgt(:,3))
axis equal
grid on
legend("Satellite","Sat ground track","LoS")

%% Define the satellite scenario for visualization
% Set simulation duration to 1 full orbital period


sampleTime = 60;
stopTime = startTime + seconds(simLength);
sc = satelliteScenario(startTime,stopTime,sampleTime);

% Add satellite
sat = satellite(sc,Rsat_ts,"Name","CubeSat");
pointAt(sat,Qin2body_ts);
groundTrack(sat);
sat.Visual3DModel = "bus.glb";
coordinateAxes(sat);

% Play scenario
satelliteScenarioViewer(sc,"CameraReferenceFrame","Inertial");