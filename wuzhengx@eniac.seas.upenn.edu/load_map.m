function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

% Import the data 
f = fopen(filename);
% Read the data line by line.
line = fgetl(f);
blocks = [];
while ischar(line)
    if isempty(line) || line(1) == '#'
        line = fgetl(f);
        continue;
    elseif strcmp(line(1:8), 'boundary')
        boundary = str2num(line(9: end));
        line = fgetl(f);
    elseif strcmp(line(1:5), 'block')
        blocks = [blocks; str2num(line(6: end))];
        line = fgetl(f);
    end
    
end
fclose('all');

% Mesh
% Get the upper corner boundary, and lower corner boundary of the map
boundMax = [boundary(4), boundary(5), boundary(6)];
boundMin = [boundary(1), boundary(2), boundary(3)];
% Get the mesh grid length based on the boundary and the resolution of map
xLen = boundMax(1) - boundMin(1);
yLen = boundMax(2) - boundMin(2);
zLen = boundMax(3) - boundMin(3);
z_res = bsxfun(@min, boundMax(3), z_res);
% Get the actual index of the points on the grid
[x, y, z] = meshgrid(boundMin(1): xy_res: boundMax(1), boundMin(2): xy_res: boundMax(2), boundMin(3): z_res: boundMax(3));

% Remove the very last edge of the matrices
if mod(xLen, xy_res) == 0
    x(end, :, :) = [];
    y(end, :, :) = [];
    z(end, :, :) = [];
end
if mod(yLen, xy_res) == 0
    x(:, end, :) = [];
    y(:, end, :) = [];
    z(:, end, :) = [];
end
if mod(zLen, z_res) == 0
    x(:, :, end) = [];
    y(:, :, end) = [];
    z(:, :, end) = [];
end


map_size = size(x);

% define the blocks in the meshed map
if ~isempty(blocks)
    % Based on the margin, we need to first define an area for blocks
    blockNum = ones(size(blocks, 1), 1);
    blockMin = [blocks(:, 1) - margin * blockNum, blocks(:, 2) - margin * blockNum, blocks(:, 3) - margin * blockNum];
    blockMax = [blocks(:, 4) + margin * blockNum, blocks(:, 5) + margin * blockNum, blocks(:, 6) + margin * blockNum];
    
    result = zeros(map_size);

    for i = 1 : size(blocks, 1)
        block = zeros(map_size);
        
        blockMin(i, :) = bsxfun(@max, blockMin(i, :), boundMin);
        blockMax(i, :) = bsxfun(@min, blockMax(i, :), boundMax);
        
        xBlocksMin = floor((blockMin(i, 1) - boundMin(1)) / xy_res) + 1;
        yBlocksMin = floor((blockMin(i, 2) - boundMin(2)) / xy_res) + 1;
        zBlocksMin = floor((blockMin(i, 3) - boundMin(3)) / z_res) + 1;
        
        xBlocksMax = ceil((blockMax(i, 1) - boundMin(1)) / xy_res);
        yBlocksMax = ceil((blockMax(i, 2) - boundMin(2)) / xy_res);
        zBlocksMax = ceil((blockMax(i, 3) - boundMin(3)) / z_res);
        
        block(yBlocksMin: yBlocksMax, xBlocksMin: xBlocksMax, zBlocksMin: zBlocksMax) = 1;
        
        result = block | result;

    end
    xBlocks = [blockMin(:, 1), blockMax(:, 1)];
    yBlocks = [blockMin(:, 2), blockMax(:, 2)];
    zBlocks = [blockMin(:, 3), blockMax(:, 3)];
    colors = blocks(:, 7:9)./255;
else
    result = zeros(map_size); 
    xBlocks = double.empty(0, 2);
    yBlocks = double.empty(0, 2);
    zBlocks = double.empty(0, 2);
    colors = double.empty(0, 3);
end

% Reshape the matrice for output
x = bsxfun(@min, x(:) + xy_res/2, boundMax(1));
y = bsxfun(@min, y(:) + xy_res/2, boundMax(2));
z = bsxfun(@min, z(:) + z_res/2, boundMax(3));
occupied = result(:);



map{1} = [x, y, z, occupied];
map{2} = [xy_res, z_res, map_size, boundMin, boundMax];
map{3} = [xBlocks, yBlocks, zBlocks, colors];

end