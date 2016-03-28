function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%   MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%   map where a node is considered fill if it lies within 'margin' distance of
%   on abstacle.

% University Of Pennsylvania
% Zhengxuan Wu
% wuzhengx@seas.upenn.edu
% 09, Jan., 2016

%% INITIALIZE
blockNum = 0;
res = [xy_res xy_res z_res];

%% LOAD DATA
% Load txt file in the type of matrix.
block = zeros(100,9);
% File Operation
fid = fopen(filename,'r');
tline = fgetl(fid);
while ischar(tline)
    if strcmp(sscanf(tline,'%s',1),'boundary')
        boundary = sscanf(tline,'%*s %f %f %f %f %f %f')';
    elseif strcmp(sscanf(tline,'%s',1),'block')
        blockNum = blockNum + 1;
        block(blockNum,:) = sscanf(tline,'%*s %f %f %f %f %f %f %f %f %f')';
    end
    tline = fgetl(fid);
end
fclose(fid);

% From the reachable data, we get upperright and lowerleft of boundary.
map.boundary.lowerleft = boundary(1:3);
map.boundary.upperright = boundary(4:6);

% Set up the dimensions for our grid system for future usage
unitL = ceil((map.boundary.upperright - map.boundary.lowerleft)./res);
colums = unitL(1);
rows = unitL(2);
depthes = unitL(3);
dimensions = colums * rows * depthes;

% Define a function, so that, we can interchangable calculate the position
% for the points on the acutal map to our grid system
re2ary = @(position) bsxfun(@min, bsxfun(@max,ceil(bsxfun(@rdivide, ...
                         bsxfun(@minus, position, map.boundary.lowerleft), ...
                         res)), 1), unitL);

% Define the discrete space array.
grid = (map.boundary.upperright-map.boundary.lowerleft)./ unitL;

% Initialize discrete 3-D map.
mapdata = zeros(unitL);

% We created a 0, 1 map for representing the blocks
for i = 1: blockNum
    map.block{i}.lowerleft = block(i,1:3);
    map.block{i}.upperright = block(i,4:6);
    map.block{i}.color = block(i,7:9)/255; 
end

% Set the block units in discrete 3D map to be 1.
for i = 1: blockNum
	upperR = re2ary(map.block{i}.upperright + margin);
	lowerL = re2ary(map.block{i}.lowerleft - margin);
    mapdata(lowerL(1):upperR(1), lowerL(2):upperR(2), lowerL(3):upperR(3)) = 1;
end

%% EXTERNAL FUNCTIONS
% Define a function, so that, we can interchangable calculate the position
% for the points on the grid system to our actual map
map.ary2re = @ (gridCoordinate) bsxfun(@plus, bsxfun(@times, gridCoordinate - .5, grid), map.boundary.lowerleft);

% Define struct array for neighbor array increment.
[nghbX,nghbY,nghbZ] = meshgrid(-1:1, -1:1, -1:1);
nghbAryInc = [nghbX(:),nghbY(:),nghbZ(:)];
map.nghbAryInc = nghbAryInc(any(nghbAryInc,2),:);

% Define a function that finds neighbour points for one position.
map.nghbFind = @findNghb;

%% LOAD DATA TO STRUCT FOR MAP
map.blockNum = blockNum;
map.res = res;
map.margin = margin;
map.unitL = unitL;
map.colums = colums;
map.rows = rows;
map.depthes = depthes;
map.dimensions = dimensions;
map.re2ary = re2ary;
map.map = mapdata;
end

%% SUB FUNCTION
function nghbDat = findNghb(map,positionAry)
% Find all the possible neighbors for a position.
nghbDat = bsxfun(@plus, map.nghbAryInc, positionAry);

% Delete position neighbors when they are out of the boundary of the map.
nghbDat = nghbDat((nghbDat(:,1) >= 1) & (nghbDat(:,2) >= 1) & (nghbDat(:,3) >= 1) & ...
		  (nghbDat(:,1) <= map.colums) & (nghbDat(:,2) <= map.rows) & (nghbDat(:,3) <= map.depthes), : );
nghbDat = nghbDat(~any(collide(map, map.ary2re(nghbDat)),2), :);
end