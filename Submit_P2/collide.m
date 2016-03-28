function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

% point = ones(size(map{1}, 1), 1);
xy_res = map{2}(1);
z_res = map{2}(2);
x_size = map{2}(4);
y_size = map{2}(3);
x_min = map{2}(6);
y_min = map{2}(7);
z_min = map{2}(8);
map_size = map{2}(3:5);
C = zeros(size(points, 1), 1);
for i = 1: size(points, 1)

x_index = floor((points(i, 1) - x_min) / xy_res) + 1;
y_index = floor((points(i, 2) - y_min) / xy_res) + 1;
z_index = floor((points(i, 3) - z_min) / z_res) + 1;
index = y_index + (x_index - 1) * y_size + (z_index - 1) * x_size * y_size;
%     index = sub2ind(map_size, y_index, x_index, z_index);
C(i) = map{1}(index, 4);
end
end