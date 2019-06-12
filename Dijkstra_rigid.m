clear all
clc

global cmap;
cmap = [1 1 1; ...  % 1 - White - map
    0 0 0; ...      % 2 - Black - obstacle
    1 0 0; ...      % 3 - Red - Destination
    1 0.5 0; ...    % 4 - Orange - Closed List
    0 1 0; ...      % 5 - Green - Start Node
    1 1 0; ...      % 6 - Yellow - Open List
    0 0 1];         % 7 - Blue - Final Path

colormap(cmap);

Dijkstra ();

function [map,X,Y] = CreateMap()
global cmap
res = 1;
x = 0:res:250;
y = 0:res:250;
[X,Y] = meshgrid(x,y);
cols = size(y,2);
rows = size(x,2);
map = zeros(cols,rows);
obs = zeros(cols,rows);
rad = input('Enter the radius of the Robot: ');
clr = input('Enter the clearance of the Robot: ');
gap = ceil(clr+rad);

circle = (X-190).^2 +(Y-130).^2 <=225;
rectangle = (X<100)&(X>50)&(Y<112.5)&(Y>67.5);
ellipse = ((X-140).^2/225) + ((Y-120).^2/36) <=1;
polygon = (Y>15)&(37*X-20*Y<6101)&(38*X+23*Y<8530)&(38*X-7*Y>5830)&(X>163);
polygon1 =(2*X+19*Y<1314)&(41*X+25*Y>6525)&(X<164)&(Y>15);

circle2 = (X-190).^2 +(Y-130).^2 <= (15+gap)^2;
circle2= circle2-circle;


new_major = 15+gap;
new_minor = 6+gap;
ellipse2 = ((X-140).^2/(new_major)^2) + ((Y-120).^2/(new_minor)^2) <=1;
ellipse2 = ellipse2 - ellipse;

circle1 = (X-50).^2 +(Y-67.5).^2 <=gap^2;
circle_2 = (X-50).^2 +(Y-112.5).^2 <=gap^2;
circle3 = (X-100).^2 +(Y-67.5).^2 <=gap^2;
circle4 = (X-100).^2 +(Y-112.5).^2 <=gap^2;
rect_1 = (X>50)&(X<100)&(Y>112.5)&(Y<=112.5+gap);
rect_2 = (X>50)&(X<100)&(Y>67.5-gap)&(Y<67.5);
rect_3 = (X>50-gap)&(X<50)&(Y>67.5)&(Y<112.5);
rect_4 = (X>100)&(X<100+gap)&(Y>67.5)&(Y<112.5);
rect1 = (X >= 50)&(X<=100)&(Y>=67.5)&(Y<112.5);

circle5 = (X-173).^2 +(Y-15).^2 <=gap^2;
circle6 = (X-193).^2 +(Y-52).^2 <=gap^2;
circle7 = (X-170).^2 +(Y-90).^2 <=gap^2;
circle8 = (X-1693).^2 +(Y-52).^2 <=gap^2;
circle9 = (X-125).^2 +(Y-56).^2 <=gap^2;
circle10 = (X-150).^2 +(Y-15).^2 <=gap^2;
circle11 = (X-163).^2 +(Y-52).^2 <=gap^2;
rect5 = (Y>=(16-gap)) & (Y<=15)& (X>150)& (X<173);

del = ceil(gap*sind(45));
dx=gap;
dy=1;
x1 = 173+dx;
y1 = 15-dy;
c_1 = -20*y1 + 37*x1;
rect6 =  (37*X-20*Y>=6101)&(37*X-20*Y<=c_1)&(Y>=15-del)&(Y<=52-del);

dx=gap;
dy=1;
x1 = 193+dx;
y1 = 52+dy;
c_2  = 23*y1 + 38*x1;
rect7 =  (38*X+23*Y>=8530)&(38*X+23*Y<=c_2)&(Y>=52+del)&(Y<=90+del);

x1 = x1-dx;
y1 = y1+dy;
c_3 = (-7)*y1 + 38*x1;
rect8 =  (38*X-7*Y<=5830)&(38*X-7*Y>=c_3)&(Y<=90+del)&(Y>=52+del);

c_4 = line(2,19,163,52,gap);
rect9 =  (2*X+19*Y>=1314)&(2*X+19*Y<=c_4)&(X<=163+del)&(X>=125-del);

dx=gap;
dy=1;
x1 = 125-dx;
y1 = 56-dy;
coeff5 = 25*y1 + 41*x1;
r10 = (41*X+25*Y<=6525)&(41*X+25*Y>=coeff5)&(Y>=15-del)&(Y<=56-del);

enlarged = circle2|ellipse2|circle1|circle_2|circle3|circle4|rect_1|rect_2|rect_3|rect_4|circle5|circle6|circle7|circle8|circle9|circle10|circle11|rect5|rect6|rect7|rect8|rect9|r10|rect1;
obs = circle|rectangle|ellipse|polygon|polygon1|enlarged;
map(obs) = 2;

end

function DrawMap(map)
global cmap
image(map)
set(gca, 'ydir', 'normal')
set(gca, 'xtick', 0:5:250)
set(gca, 'ytick', 0:5:150)
xlabel('X in mm')
ylabel('Y in mm')
grid on;
axis image;
axis([0 250 0 150])
end

function Dijkstra()
global cmap
[map,X,Y] = CreateMap();
[rows,cols] = size(Y);
startx = input('Enter the start x coordinate: ');
starty = input('Enter the start y coordinate: ');
destx = input('Enter destination x coordinate: ');
desty = input('Enter destination y coordinate: ');


if (isa(startx,'double') == 1)
    startx = round(startx);
end

if (isa(starty,'double') == 1)
    starty = round(starty);
end

if (isa(desty,'double') == 1)
    desty = round(desty);
end

if (isa(destx,'double') == 1)
    destx = round(destx);
end

if (startx == 0)
    startx = startx + 1;
end

if (starty == 0)
    starty = starty + 1;
end

if (destx > 250)
    if (desty > 250)
        desty = input('Out of bounds. Enter y coordinate again: ');
    end
    startx = input('Enter the start x coordinate: ');
end
% Checking if destination or start coordinates inside obsracle space
start_lin = sub2ind(size(Y), starty, startx);
dest_lin  = sub2ind(size(Y), desty,  destx);

while(map(start_lin) == 2)
    startx = input('Enter the start x coordinate: ');
    starty = input('Enter the start y coordinate: ');
end
while(map(dest_lin) == 2)
    destx = input('Enter destination x coordinate: ');
    desty = input('Enter destination y coordinate: ');
end

% Generate linear indices of start and dest nodes

start_lin = sub2ind(size(Y), starty, startx);
dest_lin  = sub2ind(size(Y), desty,  destx);

% Defining the parent matrix
parent_node = zeros(rows,cols);

% Initializing F and G arrays with infinite cost
f = Inf(rows,cols);
g = Inf(rows,cols);
g(start_lin) = 0;
f(start_lin) = 0;

while true

    map(start_lin) = 5;
    map(dest_lin) = 3;

    % Drawing the path continuously
%     if (true)
%         DrawMap(map)
%         drawnow;
%     end

    % Find the node with the minimum f value
    [minimumf, current_node] = min(f(:));

    % Condition to check for Destination node
    if ((current_node == dest_lin) || isinf(minimumf))
        break;
    end;

    map(current_node) = 4;
    [i, j] = ind2sub(size(f), current_node);

     if (i-1 > 1) %% Down
        if (map(i-1,j)~=2 && map(i-1,j)~=5)
            if g(i-1,j) > (g(i,j) + 1)
                g(i-1,j) = g(i,j) + 1;
                f(i-1,j) = g(i-1,j);
                map(i-1,j) = 6;
                parent_node(i-1,j) = current_node;
            end
        end
    end
    if (i+1 < rows) % Up
        if (map(i+1,j)~=2 && map(i+1,j)~=5)
            if g(i+1,j) > (g(i,j) + 1)
                g(i+1,j) = g(i,j) + 1;
                f(i+1,j) = g(i+1,j);
                map(i+1,j) = 6;
                parent_node(i+1,j) = current_node;
            end
        end
    end
    if (j-1 > 1 ) % LEFT
        if (map(i,j-1)~=2 && map(i,j-1)~=5)
            if g(i,j-1) > (g(i,j) + 1)
                g(i,j-1) = g(i,j) + 1;
                f(i,j-1) = g(i,j-1);
                map(i,j-1) = 6;
                parent_node(i,j-1) = current_node;
            end
        end
    end
    if (j+1 < cols) %RIGHT
        if (map(i,j+1)~=2 && map(i,j+1)~=5)
            if g(i,j+1) > (g(i,j) + 1)
               g(i,j+1) = g(i,j) + 1;
                f(i,j+1) = g(i,j+1);
                map(i,j+1) = 6;
                parent_node(i,j+1) = current_node;
            end
        end
    end

    if (i-1 > 1 && j+1 < cols) % Diagonal down right
        if (map(i-1,j+1)~=2 && map(i-1,j+1)~=5)
            if g(i-1,j+1) > (g(i,j) + sqrt(2))
                g(i-1,j+1) = g(i,j) + sqrt(2);
                f(i-1,j+1) = g(i-1,j+1);
                map(i-1,j+1) = 6;
                parent_node(i-1,j+1) = current_node;
            end
        end
    end

     if (i+1 < rows && j+1 < cols) % Diagonal up right
        if (map(i+1,j+1)~=2 && map(i+1,j+1)~=5)
            if g(i+1,j+1) > (g(i,j) + sqrt(2))
                g(i+1,j+1) = g(i,j) + sqrt(2);
                f(i+1,j+1) = g(i+1,j+1);
                map(i+1,j+1) = 6;
                parent_node(i+1,j+1) = current_node;
            end
        end
     end

    if (i+1 < rows && j-1 > 1) % Diagonal up left
        if (map(i+1,j-1)~=2 && map(i+1,j-1)~=5)
            if g(i+1,j-1) > (g(i,j) + sqrt(2))
                g(i+1,j-1) = g(i,j) + sqrt(2);
                f(i+1,j-1) = g(i+1,j-1);
                map(i+1,j-1) = 6;
                parent_node(i+1,j-1) = current_node;
            end
        end
    end

    if (i-1 > 1 && j-1 > 1) % Diagonal down left
      if (map(i-1,j-1)~=2 && map(i-1,j-1)~=5)
          if g(i-1,j-1) > (g(i,j) + sqrt(2))
              g(i-1,j-1) = g(i,j) + sqrt(2);
              f(i-1,j-1) = g(i-1,j-1);
              map(i-1,j-1) = 6;
              parent_node(i-1,j-1) = current_node;
          end
      end
    end

    f(current_node) = Inf;

end
% Constructing the route
if (isinf(f(dest_lin)))
    route = [];
else
    route = [dest_lin];

    while (parent_node(route(1)) ~= 0)
        route = [parent_node(route(1)), route];
    end

    % Drawing the path
    for k = 2:length(route) - 1
        map(route(k)) = 7; % giving it a grey colour
    end
end
DrawMap(map)
end

function ans = line(x_c, y_c, x1,y1,gap)
dx=1;
dy=gap;
x1 = x1-dx;
y1 = y1+dy;
ans = y_c*y1 + x_c*x1;
end
