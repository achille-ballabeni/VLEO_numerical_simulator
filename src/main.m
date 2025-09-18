clear
%% Orbital parameters and inital conditions
Re = earthRadius;
orbital_parameters = [Re+3000e3;0.3;0;0;0;0];
initial_attitude = [0;0;0];
initial_angular_velocity = [0;0;0];
startTime = datetime(2020,1,1,12,0,0);

%% Run simulation
cubesat = satellite_simulation(orbital_parameters,initial_attitude,initial_angular_velocity,startTime);
cubesat.initialize_model("simulink/satellite_propagator.slx");
cubesat.simulate();
cubesat.play_scenario();