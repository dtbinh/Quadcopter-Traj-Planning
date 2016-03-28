function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

% calling the plot map function to plot out the actual map
% this function is implemented in a different m file
% University Of Pennsylvania
% Zhengxuan Wu
% wuzhengx@seas.upenn.edu
% 09, Jan., 2016

% University Of Pennsylvania
% Zhengxuan Wu
% wuzhengx@seas.upenn.edu
% 09, Jan., 2016

% Load data from the struct.
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;

ind = 1;
while ind<=blockNum
matrixplot = find_vertex(map,ind);
% Plot the block.
% using the built in function of plotting the block
patch('Vertices',matrixplot,'Faces',[1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8],'FaceColor',block{ind}.color)
ind = ind + 1;
end
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;
% Load data with margin into matrixplot.
mapp.blockNum = blockNum;
aaa = 1;
while aaa == 1
for i = 1: blockNum
mapp.block{i}.lowerleft = block{i}.lowerleft - margin;
mapp.block{i}.upperright = block{i}.upperright + margin;
vertexMat = find_vertex(mapp,i);
% Plot the block margin.
patch('Vertices',vertexMat,'Faces',[1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8],'FaceColor',block{i}.color,'FaceAlpha',0.5,'EdgeColor','none')
end
aaa = 0;
end
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;
% Open figure 1.
figure(10);
axis equal
axis([ boundary.lowerleft(1) boundary.upperright(1)...
      boundary.lowerleft(2) boundary.upperright(2)...
      boundary.lowerleft(3) boundary.upperright(3)]);
grid on

end

function matrixplot = find_vertex(map,iteration)
% FIND_VERTEXMAT Find iall vertice for blocks.
% Load struct to variable.
block = map.block;
blockNum = map.blockNum;

% Initialization
xLength = zeros(blockNum,1);
yLength = zeros(blockNum,1);
matrixplot = zeros(8, 3);

% Define the side length.
xLength(iteration) = block{iteration}.upperright(1)-block{iteration}.lowerleft(1);
yLength(iteration) = block{iteration}.upperright(2)-block{iteration}.lowerleft(2);

% Define vertex and load data into matrixplot.
index = 1;
while index
matrixplot(1,:) = block{iteration}.lowerleft;
matrixplot(2,:) = block{iteration}.lowerleft + [xLength(iteration) 0 0];
matrixplot(3,:) = block{iteration}.lowerleft +[xLength(iteration) yLength(iteration) 0];
matrixplot(4,:) = block{iteration}.lowerleft +[0 yLength(iteration) 0];
matrixplot(5,:) = block{iteration}.upperright + [-xLength(iteration) -yLength(iteration) 0];
matrixplot(6,:) = block{iteration}.upperright + [0 -yLength(iteration) 0];
matrixplot(7,:) = block{iteration}.upperright;
matrixplot(8,:) = block{iteration}.upperright + [-xLength(iteration) 0 0];
index = 0;
end
hold on

%% PLOT THE PATH
plot3(path(:,1), path(:,2), path(:,3), 'y', 'LineWidth', 10);
end