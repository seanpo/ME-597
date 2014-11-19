function [] = RunRobot( plot_title, v, sigma, motion_disturbance_coefficient, starting_robot_state, sampling_period, sim_duration )
% Runs a robot for given wheel velocities, sampling frequency
% simulation duration, and deltas for each wheel for each iteration
%
% Usage: RunRobot( plot_title, w1, w2, [motion_disturbance_coefficient, 
% starting_robot_state, sampling_period, sim_duration])
%
% plot_title: Title of the plot
% v: Wheel one input
% sigma: Wheel two input
% motion_disturbance_coefficient: Determines the effect of the disturbance(1)
% starting_robot_state: Determines the starting state of the robot ([0; 0; 0])
% sampling_period: Determines the simulated sampling period (0.1)
% sim_duration: Determines the length of the simulation in sim time seconds. (20)

%% Default variables 
    l = 0.3;      % Length of the robot = 0.3 m

%% Input verification
    if nargin < 3 || isempty(v) || isempty(sigma)
       error('Wheel speed parameters are required and cannot be empty.')
    end

    if nargin < 4 || isempty(motion_disturbance_coefficient)
       motion_disturbance_coefficient = 1;
    end

    if nargin < 5 || isempty(starting_robot_state)
       starting_robot_state = [-1; -1; 0];
    end

    if nargin < 6 || isempty(sampling_period)
       sampling_period = 0.1;
    end

    if nargin < 7 || isempty(sim_duration)
       sim_duration = 20; 
    end

%% Disturbance Models

    % Motion Disturbance Model
    R = [0.02^2 0 0;
        0 0.02^2 0;
        0 0 degtorad(1)^2];
    [RE, Re] = eig(R);
  
 
%% Motion Model (Ax+Bu+e)
    A = eye(3);
    
%% Initialize Simulation
    t = 0:sampling_period:sim_duration; 
    input = [v; sigma];

    vehicle_state = zeros(3, length(t));   % [x; y; theta]
    front_vehicle_state = zeros(2, length(t));   % [xf; yf]
    vehicle_state(:, 1) = starting_robot_state;        
    front_vehicle_state(:, 1) = ...
            [
                starting_robot_state(1) + l*cos(starting_robot_state(3));
                starting_robot_state(2) + l*sin(starting_robot_state(3));
            ];

%% Simulate model
    for i=2:length(t)
        % Motion Disturbance
        motion_disturbance = Disturbance(RE, Re);
        % Transformation Matrix (from Body Frame to Inertial Frame)
        theta = vehicle_state(3,i-1);
        rotation_input = min(max(degtorad(input(2) - t(i)), -30), 30);
        delta = ...
            [
                input(1)*cos(theta);
                input(1)*sin(theta);
                input(1)*tan(rotation_input)/l;
            ] * sampling_period;

        % Motion Model Xt = Xt-1 + BUt + e
        vehicle_state(:,i) = A*vehicle_state(:,i-1) + delta + motion_disturbance*motion_disturbance_coefficient;
        front_vehicle_state(:, i) = ...
            [
                vehicle_state(1, i) + l*cos(vehicle_state(3, i));
                vehicle_state(2, i) + l*sin(vehicle_state(3, i))
            ];
        
        % Plot
        figure(1); clf;
        plot(vehicle_state(1,1:i), vehicle_state(2,1:i),'ro'); hold on;
        plot(front_vehicle_state(1,1:i), front_vehicle_state(2,1:i),'bo'); 
        hold off;
        grid=40; axis([-grid grid -grid grid]);
        xlabel('X Intertial Frame (m)'); ylabel('Y Inertial Frame (m)');
        title(plot_title);   
    end    
end

