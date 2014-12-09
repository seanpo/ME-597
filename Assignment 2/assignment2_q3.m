clear; clc;

%% Planning map

disp('Bike Calculating Paths');

I = imread('IGVCmap.jpg');
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size

% Define single freespace nonconvex polygon by its vertices, each hole
% separated by NaNs
env = [0 0; 0 N; M N; M 0; 0 0];

% Robot start position
dxy = 0.1;
startpos = [40/dxy 5/dxy pi];
p0 = [startpos(1) startpos(2)];

% Target location
searchgoal = [50 10]/dxy;

% Get milestones
nS = 2000;
samples = [M*rand(nS,1) N*rand(nS,1)];
%keep = inpolygon(samples(:,1),samples(:,2), env(:,1),env(:,2));
keep=[];
for i=1:nS
    % if map=1 within 30cm range of a sample, drop the point
    % ceiling to limit the indices: 0<x<=M, 0<y<=N
    if map(ceil(samples(i,1)), ceil(samples(i,2))) == 0
        keep = [keep; samples(i,1) samples(i,2)];
    end
end
milestones = [p0; searchgoal; keep];
nM = length(milestones(:,1));

% Plotting
figure(1); clf; hold on;
colormap('gray');
imagesc(1-map');
plot(startpos(1), startpos(2), 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(searchgoal(1), searchgoal(2), 'gx', 'MarkerSize',10, 'LineWidth', 3 );
plot(samples(:,1),samples(:,2),'y.');
plot(milestones(:,1),milestones(:,2),'m.');
axis equal;

% Attempt to add closest p edges
p = 30;
e = zeros(nM,nM);
D = zeros*ones(nM,nM);
for i = 1:nM
    % Find closest neighbours
    for j = 1:nM
        d(j) = norm(milestones(i,:)-milestones(j,:));
    end
    [d2,ind] = sort(d);
    % Check for edge collisions (no need to check if entire edge is
    % contained in obstacles as both endpoints are in free space)
    for j=1:p
        cur = ind(j);
        if (i<cur)
            if (~checkCollision(milestones(i,:),milestones(cur,:), map, M, N))
                e(i,cur) = 1;
                e(cur,i) = 1;
                plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m');
            end
        end
    end
end

% Find shortest path
[sp, sd] = shortestpath(milestones, e, 1, 2);
for i=1:length(sp)-1
    plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
end

waypoints=[];
for i=1:length(sp)
    waypoints=[waypoints; milestones(sp(i),1) milestones(sp(i),2)];
end

%% Path Execution

disp('Bike Tracing the Shortest Path');

% Given Constants
l = 0.3;        % L = 30 cm
xStdDev = 0.02;   % standard deviation of x and y = 0.02 m
yStdDev = 0.02;
angStdDev = degtorad(1);   % standard deviation of theta = 1 deg
delta_max = degtorad(30); % Theta limit = +/- 30 deg
delta_min = degtorad(-30);
v = 3.0;                  % speed input at 3m/s
k = 1;          % gain

% Time Span
dt = 0.1;       % update interval of 0.1s

% Motion Model
x = startpos;       %initial state

% Motion disturbance model
R = [xStdDev^2 0 0;         
    0 yStdDev^2 0;
    0 0 angStdDev^2];
[RE,Re] = eig(R);

% init input
u=[0; 0];

% front wheel location
p = [];     % front wheel initialization
p = [p; x(1)+l*cos(x(3)) x(2)+l*sin(x(3))];

% Set Carrot
if length(waypoints)>0
    c_x = waypoints(1,1);
    c_y = waypoints(1,2);
    carrot = [c_x; c_y];

    % error update
    des_bearing = atan2((carrot(2)-p(2)),(carrot(1)-p(1)));
    % error = desired bearing - actual bearing
    error = wrapToPi(des_bearing(1) - x(3));
    delta = k*error;
    % requires a saturator
    if delta > delta_max
        delta = delta_max;
    elseif delta < delta_min
        delta = delta_min;
    end

    u = [v; delta];

    i=2;
    j=2;
    while sqrt((p(i-1,1)-waypoints(length(waypoints),1))^2+(p(i-1,2)-waypoints(length(waypoints),2))^2) > 5

        e = RE*sqrt(Re)*randn(3,1); % Motion disturbance

        % apply the Motion model
        x(1) = x(1) + u(1)*cos(x(3))*dt + e(1);
        x(2) = x(2) + u(1)*sin(x(3))*dt + e(2);
        x(3) = wrapToPi(x(3) + u(1)*tan(u(2))/l*dt + e(3));

    % update
        % front wheel position update
        p = [p; x(1)+l*cos(x(3)) x(2)+l*sin(x(3))];

        % update carrot
        r=5;

        if sqrt((p(i,1)-waypoints(j,1))^2+(p(i,2)-waypoints(j,2))^2) < 5 && j<length(waypoints)
            j=j+1;
        end

        th = atan2((waypoints(j,2)-p(i,2)),(waypoints(j,1)-p(i,1)));
        c_x = p(i,1) + r*cos(th);
        c_y = p(i,2) + r*sin(th);
        carrot = [c_x; c_y];

        % error update
        des_bearing = atan2((carrot(2)-p(i,2)),(carrot(1)-p(i,1)));    
        error = wrapToPi(des_bearing - x(3));
        delta = k*error;
        % requires a saturator
        if delta > delta_max
            delta = delta_max;
        elseif delta < delta_min
            delta = delta_min;
        end

        % input update
        u = [v; delta];

    % Plot
        if mod(i,10) == 0
            plot(p(1:i,1),p(1:i,2),'b.--', 'LineWidth',3);
            plot(carrot(1), carrot(2),'k.', 'LineWidth',3);
        end
        i=i+1;
    end
else
    disp('No Routes Found');
end