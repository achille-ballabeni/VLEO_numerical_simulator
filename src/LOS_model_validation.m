%% Orbital parameters
e = 0;
Re = 6378; % [km]
a = Re + 500;
mi = 398600.418; % [km^3/s^2] 
T = 2 * pi * sqrt(a^3 / mi); % [s]

%% Define the satellite scenario
% Set simulation duration to 1 full orbital period
startTime = datetime(2025,08,24);
stopTime = startTime + seconds(T);
sampleTime = 60;

sc = satelliteScenario(startTime,stopTime,sampleTime);
a = a*1000;
sat = satellite(sc,a,e,0,0,0,0);
sat.Visual3DModel = "bus.glb";
coordinateAxes(sat);



% Play scenario
satelliteScenarioViewer(sc)