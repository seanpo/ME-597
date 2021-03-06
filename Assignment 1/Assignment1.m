%% Assignment 1
clear; clc; close all;
    
%% Q2 
    % Given inputs
    RunRobot( ...
        'Robot motion with w1=-1.5 rad/s w2=2.0 rad/s and w3=1.0 rad/s', ...
        -1.5, 2.0, 1.0, 0 ...
    )
    % Run in a straight line
    RunRobot( ...
        'Straight line motion (w1=-2.0 rad/s w2=2.0 rad/s and w3=0 rad/s)', ...
        -2.0, 2.0, 0, 0 ...
    )
    % Rotate in a perfect circle
    r = 0.25; 
    l = 0.3; 
    circular_motion = inv( ...
                            r*[0         -2/3*cos(pi/6)      2/3*cos(pi/6);
                            2/3        -2/3*sin(pi/6)      -2/3*sin(pi/6);
                            1/(3*l)    1/(3*l)             1/(3*l)] ...
                         ) * [2*pi/15; 0; 2*pi/15];
 
    RunRobot( ...
        strcat( ... 
            'Circular motion with radius 2 in 15 seconds (w1=', ... 
            num2str(circular_motion(1)), ...
            'rad/s w2=', num2str(circular_motion(2)), 'rad/s and w3=', ...
            num2str(circular_motion(3)), 'rad/s)' ...
        ), ...
        circular_motion(1), circular_motion(2), circular_motion(3), 0 ...
    )
    % Rotate in expanding circle
    RunRobot( ...
        'Straight expanding circular motion', ...
        -2.0, -2.0, -2.0, 0, 0, 0.1, 0, [0; 0; 0], 0.1, 20 ...
    )
    
%% Q5
    % Given inputs
    RunRobot( ...
        'Robot motion with w1=-1.5 rad/s w2=2.0 rad/s and w3=1.0 rad/s', ...
        -1.5, 2.0, 1.0, 1 ...
    )