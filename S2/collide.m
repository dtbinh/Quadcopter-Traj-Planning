function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector; 
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

%% INITIALIZE
pointsNum = size(points,1);
C = zeros(pointsNum,1);
mapdataSample = map.map;

pointsDisc = map.re2ary(points);

for i = 1: pointsNum
    C(i) = mapdataSample(pointsDisc(i,1),pointsDisc(i,2),pointsDisc(i,3));
end

end