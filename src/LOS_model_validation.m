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
w0 = [0,360/T,0]; % Angular velocity in body frame [deg/s]
startTime = datetime(2020,1,1,12,0,0);
startTimeJD = juliandate(startTime);
simLength = T;

%% Setup simulation
model = "satellite_propagator.slx";
simIn = Simulink.SimulationInput(model);
simIn = simIn.setModelParameter("StopTime", num2str(simLength), ...
    "Solver","ode4", ...
    "FixedStep","1");

%% Run simulation
simOut = sim(simIn);
t = simOut.tout;

%% Do some post processing
Rsat = simOut.yout{1}.Values;
Qin2body = simOut.yout{4}.Values;
% Inverse quaternion to go from body to inertial
Qbody2in = quatinv(Qin2body.Data);
% Find direction of line of sight, considered as exiting from the x axis of
% the satellite.
LOS_hat = quatrotate(Qbody2in,[-1,0,0]);
% Intersection between line of sight and earth surface
rho = -dot(LOS_hat,Rsat.Data,2) - sqrt((dot(LOS_hat,Rsat.Data,2)).^2 - vecnorm(Rsat.Data,2,2).^2 + Re^2);
rho(imag(rho) ~= 0) = 0;
rho(rho<0) = 0;
indexes = rho ~= 0;
% Find the ground track vector
Rtar = rho.*LOS_hat;
Rgt(indexes,:) = Rsat.Data(indexes,:) + Rtar(indexes,:);
% Quick plot of the orbit to check
plot3(Rgt(:,1),Rgt(:,2),Rgt(:,3))
hold on 
plot3(Rsat.Data(:,1),Rsat.Data(:,2),Rsat.Data(:,3))
axis equal
grid on


%% Define the satellite scenario for visualization
% Set simulation duration to 1 full orbital period


sampleTime = 60;
stopTime = startTime + seconds(simLength);
sc = satelliteScenario(startTime,stopTime,sampleTime);

% Add satellite
sat = satellite(sc,Rsat,"Name","CubeSat");
pointAt(sat,Qin2body);
sat.Visual3DModel = "bus.glb";
coordinateAxes(sat);

% Play scenario
satelliteScenarioViewer(sc,"CameraReferenceFrame","Inertial");