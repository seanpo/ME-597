clear; clc;

%% Given Constants
L = 0.3;        % L = 30 cm
xStdDev = 0.02;   % standard deviation of x and y = 0.02 m
yStdDev = 0.02;
angStdDev = 1*pi/180;   % standard deviation of theta = 1 deg
% Theta limit = +/- 30 deg
v = 3;                  % speed input at 3m/s
angle = degtorad(10);      % steering angle 10 - t degrees

%% Time Span
dt = 0.1;       % update interval of 0.1s
t = 0:dt:20;    % simulation for 20 seconds

%% Motion Model
x = zeros(3,length(t));     % initialize the states for the time span
u = [v*ones(1,length(t));        % input
    angle*ones(1,length(t))];

R = [xStdDev^2 0 0;         % Motion disturbance model
    0 yStdDev^2 0;
    0 0 angStdDev^2];
[RE,Re] = eig(R);

for i=2:length(t)
    
    u(2,i) = u(2,1) - degtorad(i*dt);
    
    e = RE*sqrt(Re)*randn(3,1); % Motion disturbance
    
    % apply the Motion model
    x(1,i) = x(1,i-1) + u(1,i)*cos(x(3,i-1))*dt + e(1);
    x(2,i) = x(2,i-1) + u(1,i)*sin(x(3,i-1))*dt + e(2);
    x(3,i) = x(3,i-1) + u(1,i)*tan(u(2,i))/L*dt + e(3);
    % bound the bearing within [0, 2*pi]
    if x(3,i) > 2*pi
        x(3,i)=mod(x(3,i),2*pi);
    elseif x(3,i) < 0
        x(3,i) = x(3,i)*-1;
        x(3,i)=mod(x(3,i),2*pi);
        x(3,i) = x(3,i)*-1 + 2*pi;
    end
    
    % Plot
    figure(1);clf;
    plot(x(1,1:i),x(2,1:i),'ro');
    axis equal;
    title('Simulated Bicycle Motion');
    
end