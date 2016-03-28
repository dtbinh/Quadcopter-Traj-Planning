function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector; 
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

%% INITIALIZE
pntsNum = size(points,1);
C = zeros(pntsNum,1);
mapdata = map.map;

%% LOAD DATA
% Discretize points into discrete coordinates.
pointsD = map.re2ary(points);

% Load collide data into Matrix C.
for i = 1: pntsNum
    
    C(i) = mapdata(pointsD(i,1),pointsD(i,2),pointsD(i,3));
end

end