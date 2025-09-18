classdef satellite_simulation < handle
    % SATELLITE Satellite class to initialize, simulate, perform analysis and
    % visualize the orbit of a satellite.

    properties (Constant)
        Re = earthRadius; % [m]
        mi = 398600.418e9; % [m^3/s^2]
    end

    properties
        orbital_parameters
        initial_attitude
        initial_angular_velocity
        startTime
        simLength
        simIn
        simOut
        t
    end

    methods
        function obj = satellite_simulation(orbital_parameters, attitude, angular_velocity, startTime)
            % SATELLITE Initialize the satellite class with initial
            % conditions.
            %
            % Input Arguments
            %   orbital_parameters - Orbital parameters [semimajor-axis, eccentricity, inclination, right ascension, arguement of pericenter, true anomaly] in meters and degrees.
            %     6-by-1 array
            %   attitude - Euler rotation angles with in the order XYZ in degrees.
            %     3-by-1 array
            %   angular_velocity - Initial angular velocity vector (ECI frame).
            %     3-by-1 array
            %   startTime - Start time of the simulation.
            %     datetime

            arguments
                orbital_parameters (6,1) double
                attitude (3,1) double
                angular_velocity (3,1) double
                startTime (1,1) datetime
            end

            obj.orbital_parameters = orbital_parameters;
            obj.initial_attitude = attitude;
            obj.initial_angular_velocity = angular_velocity;
            obj.startTime = startTime;
        end

        function obj = initialize_model(obj,model_path,duration)
            % INITIALIZE_MODEL Initializes simulink model with initial conditions.
            % Converts and extract class inputs to initial values for the 
            % simulink model.
            % 
            % Input Arguments
            %   model_path - Path to the Simulink model as a string.
            %     1-by-1 string
            %   duration - Duration of the simulation in seconds. If not
            %     provided, the function will calculate the period based on
            %     the semi-major axis.
            %     1-by-1 double
            
            arguments
                obj
                model_path (1,1) string
                duration double = [];
            end
            
            % Convert to Julian date
            startTimeJD = juliandate(obj.startTime);

            % Unpack orbital parameters
            a = obj.orbital_parameters(1); % Semi-major axis
            e = obj.orbital_parameters(2); % Eccentricity
            i = obj.orbital_parameters(3); % Inclination
            raan = obj.orbital_parameters(4); % Right Ascension of Ascending Node
            aop = obj.orbital_parameters(5); % Argument of Perigee
            ta = obj.orbital_parameters(6); % True Anomaly

            % Define simulation duration
            if duration
                obj.simLength = duration;
            else
                T = period(a,obj.mi);
                obj.simLength = T;
            end

            % Convert attiude to quaternion (assuming ZYX rotation)
            theta1 = deg2rad(obj.initial_attitude(1));
            theta2 = deg2rad(obj.initial_attitude(2));
            theta3 = deg2rad(obj.initial_attitude(3));
            q0 = angle2quat(theta3,theta2,theta1,"ZYX");

            % Initial angular velocity
            w0 = obj.initial_angular_velocity;

            % Setup simulation parameters
            obj.simIn = Simulink.SimulationInput(model_path);
            obj.simIn = obj.simIn.setModelParameter("StopTime", num2str(obj.simLength), ...
                "Solver","ode4", ...
                "FixedStep","1");
            obj.simIn = obj.simIn.setBlockParameter("satellite_propagator/Spacecraft Dynamics", "startDate", num2str(startTimeJD));
            obj.simIn = obj.simIn.setBlockParameter("satellite_propagator/Spacecraft Dynamics", "semiMajorAxis", num2str(a));
            obj.simIn = obj.simIn.setBlockParameter("satellite_propagator/Spacecraft Dynamics", "eccentricity", num2str(e));
            obj.simIn = obj.simIn.setBlockParameter("satellite_propagator/Spacecraft Dynamics", "inclination", num2str(i));
            obj.simIn = obj.simIn.setBlockParameter("satellite_propagator/Spacecraft Dynamics", "raan", num2str(raan));
            obj.simIn = obj.simIn.setBlockParameter("satellite_propagator/Spacecraft Dynamics", "argPeriapsis", num2str(aop));
            obj.simIn = obj.simIn.setBlockParameter("satellite_propagator/Spacecraft Dynamics", "trueAnomaly", num2str(ta));
            obj.simIn = obj.simIn.setBlockParameter("satellite_propagator/Spacecraft Dynamics", "attitude", mat2str(q0));
            obj.simIn = obj.simIn.setBlockParameter("satellite_propagator/Spacecraft Dynamics", "attitudeRate", mat2str(w0));

        end

        function obj = simulate(obj)
            % RUN Runs the simulink model.
            obj.simOut = sim(obj.simIn);
            obj.t = obj.simOut.tout;
        end

        function play_scenario(obj,sampleTime,Name)
            % PLAY_SCENARIO Play the simulation in a satelliteScenario.
            % Set simulation duration to 1 full orbital period
            %
            % Input Arguments
            %   sampleTime - Timestep of satellite scenario simulation (defaults to 60s).
            %     scalar
            %   Name - Name dispalyed in the simulation (CubeSat).

            arguments
                obj 
                sampleTime (1,1) double = 60
                Name (1,1) string = "CubeSat"
            end

            % Extract timeseries values
            Rsat_ts = obj.simOut.yout{1}.Values;
            Qin2body_ts = obj.simOut.yout{4}.Values;
            
            % Setup satellite scenario object
            stopTime = obj.startTime + seconds(obj.simLength);
            sc = satelliteScenario(obj.startTime,stopTime,sampleTime);
            
            % Add satellite
            sat = satellite(sc,Rsat_ts,"Name",Name);
            pointAt(sat,Qin2body_ts);
            groundTrack(sat);
            sat.Visual3DModel = "bus.glb";
            coordinateAxes(sat);
            
            % Play scenario
            satelliteScenarioViewer(sc,"CameraReferenceFrame","Inertial");
        end
    end
end