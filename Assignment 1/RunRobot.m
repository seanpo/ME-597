function [] = RunRobot( plot_title, w1, w2, w3, motion_disturbance_coefficient, del_w1, del_w2, del_w3, starting_robot_state, sampling_period, sim_duration )
% Runs a robot for given wheel velocities, sampling frequency
% simulation duration, and deltas for each wheel for each iteration
%
% Usage: RunRobot( plot_title, w1, w2, w3, [motion_disturbance_coefficient, del_w1, 
% del_w2, del_w3, starting_robot_state, sampling_period, sim_duration])
%
% plot_title: Title of the plot
% w1: Wheel one input
% w2: Wheel two input
% w3: Wheel three input
% motion_disturbance_coefficient: Determines the effect of the disturbance(1)
% del_w1: Additional wheel one input per iteration (0)
% del_w2: Additional wheel two input per iteration (0)
% del_w3: Additional wheel three input per iteration (0)
% starting_robot_state: Determines the starting state of the robot ([-1; 1; 0])
% sampling_period: Determines the simulated sampling period (0.1)
% sim_duration: Determines the length of the simulation in sim time seconds. (15)

%% Default variables 
    r = 0.25;     % radius of each wheel = 0.25 m
    l = 0.3;      % radius of the robot = 0.3 m

%% Input verification
    if nargin < 4 || isempty(w1) || isempty(w2) || isempty(w3)
       error('Wheel speed parameters are required and cannot be empty.')
    end

    if nargin < 5 || isempty(motion_disturbance_coefficient)
       motion_disturbance_coefficient = 1;
    end

    if nargin < 6 || isempty(del_w1)
       del_w1 = 0;
    end

    if nargin < 7 || isempty(del_w2)
       del_w2 = 0;
    end

    if nargin < 8 || isempty(del_w3)
       del_w3 = 0;
    end

    if nargin < 9 || isempty(starting_robot_state)
       starting_robot_state = [-1; 1; 0];
    end

    if nargin < 10 || isempty(sampling_period)
       sampling_period = 0.1;
    end

    if nargin < 11 || isempty(sim_duration)
       sim_duration = 15; 
    end

%% Disturbance Models

    % Motion Disturbance Model
    R = [0.01^2 0 0;
        0 0.01^2 0;
        0 0 0.1^2];
    [RE, Re] = eig(R);
  
    % Measurement Disturbance Model
    Q = [0.5^2 0 0;
        0 0.5^2 0;
        0 0 (10*pi/180)^2];
    [QE, Qe] = eig(Q);
    
    % Accurate Measurement Disturbance Model
    Qnew = [0.01^2 0 0;
        0 0.01^2 0;
        0 0 (10*pi/180)^2];
    [QnewE, Qnewe] = eig(Qnew);
    
%% Motion Model (Ax+Bu+e)
    A = eye(3);
    B = r * [0         -2/3*cos(pi/6)      2/3*cos(pi/6);
             2/3        -2/3*sin(pi/6)      -2/3*sin(pi/6);
             1/(3*l)    1/(3*l)             1/(3*l)] * sampling_period;
    
%% Measurement Model Matrix (Cx + d)
    C = eye(1);
 
%% Initialize Simulation
    t = 0:sampling_period:sim_duration; 
    input = [w1; w2; w3];
    del_input = [del_w1; del_w2; del_w3];

    vehicle_state = zeros(3,length(t));   % initial vehicle state
    measurements = zeros(3,length(t));   % initial measurements

    predicted_vehicle_state = starting_robot_state;
    sigma_covariance = 0.01*eye(3);

    mu_S = zeros(3,length(t));  % Belief
    mu_S(:,1) = predicted_vehicle_state;

    figure; clf;
%% Simulate model
    for i=2:length(t)
        % Motion Disturbance
        motion_disturbance = Disturbance(RE, Re);
        % Transformation Matrix (from Body Frame to Inertial Frame)
        theta = vehicle_state(3,i-1);
        rotation = [cos(theta)     -sin(theta)     0;
                    sin(theta)      cos(theta)      0;
                    0               0               1];

        input = input - del_input;

        % Motion Model Xt = Xt-1 + BUt + e
        vehicle_state(:,i) = A*vehicle_state(:,i-1) + rotation*B*input + motion_disturbance*motion_disturbance_coefficient;

        % Measurement Disturbance
        disturbance_model = Q;

        %Q6 Correction - improved Q for correction every 10 iteration
        if mod(i,10) ~= 0
            measurement_disturbance = Disturbance(QE, Qe);
        else
            measurement_disturbance = Disturbance(QnewE, Qnewe);
            disturbance_model = Qnew;
        end

        % Measurement Model Yt = CXt + d
        % the magnetometer angle has 9.7 deg offset
        measurements(:,i) = C*[vehicle_state(1,i); vehicle_state(2,i); vehicle_state(3,i)-(9.7*pi/180)] + measurement_disturbance;

        % Extended Kalman Filter
        % Prediction update
        predicted_vehicle_state(1:3) = A*predicted_vehicle_state + rotation*B*input;

        predicted_rotation = [-sin(theta)   -cos(theta)     0;
                              cos(theta)      -sin(theta)     0;
                              0               0               0];

        dB = (predicted_rotation*B*input);

        Gt = [1 0 dB(1); 0 1 dB(2); 0 0 1+dB(3)];

        sigma_covariance(1:3,1:3) = Gt*sigma_covariance(1:3,1:3)*Gt' + R;

        % Measurement update
        Ht = C;
        Kt = sigma_covariance*Ht'*inv((Ht*sigma_covariance*Ht' + disturbance_model));

        predicted_vehicle_state = predicted_vehicle_state + Kt*(measurements(:,i)-Ht*predicted_vehicle_state);
        sigma_covariance = (eye(3)-Kt*Ht)*sigma_covariance;

        % Store results
        mu_S(:,i) = predicted_vehicle_state;

        % Plot
        plot(vehicle_state(1,1:i),vehicle_state(2,1:i),'ro'); hold on;
        plot(mu_S(1,1:i),mu_S(2,1:i),'bx');
        mu_pos = [predicted_vehicle_state(1) predicted_vehicle_state(2)];
        S_pos = sigma_covariance(1:2, 1:2);
        error_ellipse(S_pos,mu_pos,0.75);
        error_ellipse(S_pos,mu_pos,0.95);
        axis equal;
        grid=3; axis([-grid grid -grid grid]);
        xlabel('X Intertial Frame (m)'); ylabel('Y Inertial Frame (m)');
        title(plot_title);   
    end    
end

