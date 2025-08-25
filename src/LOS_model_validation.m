%% Earth constants
Re = earthRadius; % [m]
mi = 398600.418e9; % [m^3/s^2]

%% Orbital parameters and initial conditions
a = Re + 500e3; % [m]
e = 0;
inclination = 0;
raan = 0; % Right Ascension of Ascending node [deg]
aop = 0; % Arguement of Pericenter [deg]
ta = 0; % True amomaly [deg]

q0 = [1,0,0,0];
w0 = [0,0,0];
T = 2 * pi * sqrt(a^3 / mi); % [s]
startTime = datetime(2020,1,1,12,0,0);
startTimeJD = juliandate(startTime);
simLength = T;

%% Setup simulation
model = "satellite_propagator.slx";
simIn = Simulink.SimulationInput(model);
simIn = simIn.setModelParameter("StopTime", num2str(simLength));

%% Run simulation
simOut = sim(simIn);
t = simOut.tout;

%% Define the satellite scenario
% Set simulation duration to 1 full orbital period

sampleTime = 60;
stopTime = startTime + seconds(simLength);
sc = satelliteScenario(startTime,stopTime,sampleTime);

% Add satellite
Rsat = simOut.yout{1}.Values;
sat = satellite(sc,Rsat);
sat.Visual3DModel = "bus.glb";
coordinateAxes(sat);

% Play scenario
satelliteScenarioViewer(sc)