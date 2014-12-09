clear; clc;

%% Given Constants
l = 0.3;        % L = 30 cm
xStdDev = 0.02;   % standard deviation of x and y = 0.02 m
yStdDev = 0.02;
angStdDev = degtorad(1);   % standard deviation of theta = 1 deg
delta_max = degtorad(30); % Theta limit = +/- 30 deg
delta_min = degtorad(-30);
v = 3.0;                  % speed input at 3m/s
k = 1;          % gain

%% Time Span
dt = 0.1;       % update interval of 0.1s
t_duration = 20;
t = 0:dt:t_duration;    % simulation for 20 seconds

%% Motion Model
x = zeros(3,length(t));     % initialize the states for the time span
x0 = [0 5 -1]';       %initial state
x(:,1) = x0;

R = [xStdDev^2 0 0;         % Motion disturbance model
    0 yStdDev^2 0;
    0 0 angStdDev^2];
[RE,Re] = eig(R);

u=zeros(2,length(t));

% front wheel location
p = zeros(2,length(t));     % front wheel initialization
p(1,1) = x(1,1)+l*cos(x(3,1));
p(2,1) = x(2,1)+l*sin(x(3,1));

%% Set Carrot
% carrot on the rectangle
rect = [2 22 22 2;
        2 2 7 7];
c_x = rect(1,1);
c_y = rect(2,1);
carrot = [c_x; c_y];

% error = desired bearing - actual bearing
error = zeros(1,length(t));
des_bearing = zeros(1,length(t));
delta=zeros(1,length(t));

% error update
des_bearing(1) = atan2((carrot(2)-p(2,1)),(carrot(1)-p(1,1)));    
error(1) = wrapToPi(des_bearing(1) - x(3,1));
delta(1) = k*error(1);
% requires a saturator
if delta(1) > delta_max
    delta(1) = delta_max;
elseif delta(1) < delta_min
    delta(1) = delta_min;
end

u = [v; delta(1)];

for i=2:length(t)
    
    e = RE*sqrt(Re)*randn(3,1); % Motion disturbance
    
    % apply the Motion model
    x(1,i) = x(1,i-1) + u(1)*cos(x(3,i-1))*dt + e(1);
    x(2,i) = x(2,i-1) + u(1)*sin(x(3,i-1))*dt + e(2);
    x(3,i) = wrapToPi(x(3,i-1) + u(1)*tan(u(2))/l*dt + e(3));
    
%% update
    % front wheel position update
    p(1,i) = x(1,i)+l*cos(x(3,i));
    p(2,i) = x(2,i)+l*sin(x(3,i));
    
    % update carrot
    r=0.5;
    if sqrt((p(1,i)-rect(1,1))^2+(p(2,i)-rect(2,1))^2) < 0.5
        line=1;
        %carrot = [rect(1,1); rect(2,1)];
    elseif sqrt((p(1,i)-rect(1,2))^2+(p(2,i)-rect(2,2))^2) < 0.5
        line=2;
        %carrot = [rect(1,3); rect(2,3)];
    elseif sqrt((p(1,i)-rect(1,3))^2+(p(2,i)-rect(2,3))^2) < 0.5
        line=3;
        %carrot = [rect(1,4); rect(2,4)];
    elseif sqrt((p(1,i)-rect(1,4))^2+(p(2,i)-rect(2,4))^2) < 0.5
        line=4;
        %carrot = [rect(1,1); rect(2,1)];
    end
    
    switch line
        case 1
            c_x = p(1,i)+r;
            c_y = rect(2,1);
        case 2
            c_x = rect(1,3);
            c_y = p(2,i)+r
        case 3
            c_x = p(1,i)-r;
            c_y = rect(2,4);
        case 4
            c_x = rect(1,1);
            c_y = p(2,i)-r;
    end
    
    carrot = [c_x; c_y];        
    
    % error update
    des_bearing(i) = atan2((carrot(2)-p(2,i)),(carrot(1)-p(1,i)));    
    error(i) = wrapToPi(des_bearing(i) - x(3,i));
    delta(i) = k*error(i);
    % requires a saturator
    if delta(i) > delta_max
        delta(i) = delta_max;
    elseif delta(i) < delta_min
        delta(i) = delta_min;
    end
            
    % input update
    u = [v; delta(i)];
    
%% Plot
    figure(1);clf;
    plot([2,22,22,2,2],[2,2,7,7,2],'b--'); hold on;
    plot(p(1,1:i),p(2,1:i),'r.--'); hold on;
    plot(carrot(1), carrot(2),'mo');
    axis([0 30 0 15]);
    title('Carrot Following Controller');
    pause(0.001);
end