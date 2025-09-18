clear
%% Orbital parameters and inital conditions
Re = earthRadius;
orbital_parameters = [Re+3000e3;0.3;0;0;0;0];
initial_attitude = [0;0;0];
initial_angular_velocity = [0;0;0];
startTime = datetime(2020,1,1,12,0,0);

%% Run simulation
cubesat = satellite_simulation(orbital_parameters,initial_attitude,initial_angular_velocity,startTime);
cubesat.initialize_model("simulink/satellite_propagator.slx","400");
cubesat.simulate();

%% Perform LOS analysis
cubesat.LOS();
% Ground track vectors in ECEF
R_gt_sat_ecef = cubesat.ground_track("satellite","ecef");
R_gt_los_ecef = cubesat.ground_track("los","ecef");
% Ground track vectors in ECI
R_gt_sat_eci = cubesat.ground_track("satellite","eci");
R_gt_los_eci = cubesat.ground_track("los","eci");

% Latitude and longitude
lla_sat = ecef2lla(R_gt_sat_ecef,0,Re);
ll_sat = lla_sat(:,1:2);
lla_los = ecef2lla(R_gt_los_ecef,0,Re);
ll_los = lla_los(:,1:2);

% Plot ground tracks
figure(1)
geoplot(ll_sat(:,1),ll_sat(:,2))
hold on
geoplot(ll_los(:,1),ll_los(:,2))
legend("Satellite","LoS")
geobasemap("satellite")

% Quick plot of the orbit to check
figure(2)
plot3(cubesat.Rsat(:,1),cubesat.Rsat(:,2),cubesat.Rsat(:,3))
hold on
plot3(R_gt_sat_eci(:,1), R_gt_sat_eci(:,2), R_gt_sat_eci(:,3))
plot3(R_gt_los_eci(:,1),R_gt_los_eci(:,2),R_gt_los_eci(:,3))
axis equal
grid on
legend("Satellite","Sat ground track","LoS")

%% Play satellite scenario
cubesat.play_scenario();