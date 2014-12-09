function [ inCollision ] = checkCollision( ptA, ptB, map, M, N )

%Matlab optmized version of Bresenham line algorithm. No loops.
%Format:
%               [x y]=bham(x1,y1,x2,y2)
%
%Input:
%               (x1,y1): Start position
%               (x2,y2): End position
%
%Output:
%               x y: the line coordinates from (x1,y1) to (x2,y2)
%
%Usage example:
%               [x y]=bham(1,1, 10,-5);
%               plot(x,y,'or');
x1=round(ptA(1)); x2=round(ptB(1));
y1=round(ptA(2)); y2=round(ptB(2));
dx=abs(x2-x1);
dy=abs(y2-y1);
steep=abs(dy)>abs(dx);
if steep t=dx;dx=dy;dy=t; end

%The main algorithm goes here.
if dy==0 
    q=zeros(dx+1,1);
else
    q=[0;diff(mod([floor(dx/2):-dy:-dy*dx+floor(dx/2)]',dx))>=0];
end

%and ends here.

if steep
    if y1<=y2 y=[y1:y2]'; else y=[y1:-1:y2]'; end
    if x1<=x2 x=x1+cumsum(q);else x=x1-cumsum(q); end
else
    if x1<=x2 x=[x1:x2]'; else x=[x1:-1:x2]'; end
    if y1<=y2 y=y1+cumsum(q);else y=y1-cumsum(q); end
end

for i=1:size(x)
    for j=1:size(y)
       % If not in the map, set measurement there and stop going further 
       if (x(i)<1||x(i)>M||y(j)<1||y(j)>N)
           inCollision=1;
           return
       % If in the map but hitting an obstacle, set measurement range and
       % stop going further
       elseif (map(x(i),y(j)))
           inCollision=1;
           return
       end
    end
end

% % Initialization
% [M,N] = size(map);
% x = ptA(1);
% y = ptA(2);
% th = atan2(ptB(2)-ptA(1), ptB(1)-ptA(1));
% rmax = sqrt((ptB(2)-ptA(2))^2+(ptB(1)-ptA(1))^2);
% meas_phi=-1:0.005:1;
% 
% % For each measurement bearing
% for i=1:length(meas_phi)
%     % For each unit step along that bearing up to max range
%    for r=1:rmax
%        % Determine the coordinates of the cell
%        xi = round(x+r*cos(th+meas_phi(i)));
%        yi = round(y+r*sin(th+meas_phi(i)));
%        % If not in the map, set measurement there and stop going further 
%        if (xi<=1||xi>=M||yi<=1||yi>=N)
%            inCollision=1;
%            return
%        % If in the map but hitting an obstacle, set measurement range and
%        % stop going further
%        elseif (map(xi,yi))
%            inCollision=1;
%            return
%        end
%    end
% end


inCollision = 0 ; % Not in collision