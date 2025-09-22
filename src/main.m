clear
%% Orbital parameters and inital conditions
Re = earthRadius;
mi = 398600.418e9;
a = Re+500e3;
orbital_parameters = [a;0;0;0;0;0];
T = period(a,mi);
initial_attitude = [0;0;0];
initial_angular_velocity = [0;0;360/T];
startTime = datetime(2020,1,1,12,0,0);

%% Run simulation
timestep = 1;
cubesat = satellite_simulation(orbital_parameters,initial_attitude,initial_angular_velocity,startTime);
cubesat.initialize_model("simulink/satellite_propagator.slx",duration=200,timestep=timestep);
cubesat.simulate();

%% Perform LOS analysis
cubesat.LOS();
% Ground track vectors in ECI
[R_gt_sat_eci, ~]  = cubesat.ground_track(type="satellite",frame="eci",model="sphere");
[R_gt_tar_eci, ~] = cubesat.ground_track(type="los",frame="eci",model="sphere");
% Ground track vectors in ECEF
[R_gt_sat_ecef, lla_sat] = cubesat.ground_track(type="satellite",frame="ecef",model="sphere");
[R_gt_tar_ecef, lla_tar] = cubesat.ground_track(type="los",frame="ecef",model="sphere");

% Latitude and longitude
ll_sat = lla_sat(:,1:2);
ll_tar = lla_tar(:,1:2);

% Plot ground tracks
figure(1)
geoplot(ll_sat(:,1),ll_sat(:,2))
hold on
geoplot(ll_tar(:,1),ll_tar(:,2))
legend("Satellite","LoS")
geobasemap("satellite")

% Quick plot of the orbit to check
figure(2)
plot3(cubesat.Rsat(:,1),cubesat.Rsat(:,2),cubesat.Rsat(:,3))
hold on
plot3(R_gt_sat_eci(:,1), R_gt_sat_eci(:,2), R_gt_sat_eci(:,3))
plot3(R_gt_tar_eci(:,1),R_gt_tar_eci(:,2),R_gt_tar_eci(:,3))
axis equal
grid on
legend("Satellite","Sat ground track","LoS")

%% Play satellite scenario
% Clean coordinates from negative altitudes
lla_tar(:,3) = 0;
cubesat.play_scenario(lla_tar,sampleTime=1);

% Compare calculated velocity with numerical derivative
Vtar_numerical = diff(cubesat.Rtar)./timestep;
Vtar = cubesat.Vtar(2:end,:);
figure(3)
plot(Vtar)
hold on
plot(Vtar_numerical)
legend("u - analytic","v - analytic","w - analytic","u - numerical","v - numerical","w - numerical")
% TODO: compute difference between the two