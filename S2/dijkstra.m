function [path, num_expanded, best_cost] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther
%   apart than the resolution of the map.
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.


% University Of Pennsylvania
% Zhengxuan Wu
% wuzhengx@seas.upenn.edu
% 09, Jan., 2016

% This script basically find the optimal path solution for flying robots
% for getting from a starting point, getting through obstacles, and finally
% reaching the final goal in a optimal way. This program can run in astar
% mode, and without astar mode. However, this program is still not
% optimized. Some advanced data structure might be introduced to reduce the
% work and memories for the path.

%% DETERMINE METHOD

% Throw exceptions
if nargin < 4
    astar = false;
end

%% INITIALIZE
% Load all the data from the created struct from the previous load_map.m
colums = map.colums;
rows = map.rows;
unitL = map.unitL;
dimensions = unitL(1)*unitL(2)*unitL(3);
re2ary = map.re2ary;
ary2re = map.ary2re;


% Define a function to calculate the cost and heuristic distance between
% points A & B.
paceLength = abs(sum(bsxfun(@minus,ary2re([2,1,1;1,2,1;1,1,2]),ary2re([1,1,1;1,1,1;1,1,1])),2));
hFun = @(A) sqrt(sum(bsxfun(@power, bsxfun(@minus, ary2re(A), goal), 2.0), 2));
% Transfer goal and start to our grid system
startA = re2ary(start);
goalA = re2ary(goal);
node = startA;
% Define function transferring sub to index and index to sub.
matrixToIndex = @ (matrxInd) matrxInd(:,1)+ (matrxInd(:,2)-1) * colums + (matrxInd(:,3)-1) * colums * rows;
indexToMatrix = @ (index) ind2sub(unitL, index);

% Initialize three space matrice: visited, reachable, parent representing
% the cost from start to each discrete point in the map.
visited = zeros(unitL);
reachable = findNgb(map,node);
exploredSet = zeros(unitL);
parent = zeros(dimensions, 1);

% Define the cost matrix the initial value is infinite.
cost = inf(dimensions, 1);
cost(matrixToIndex(node)) = 0;

%% FIND THE GOAL
if astar == true
    while any(size(reachable,1))&& ~visited(goalA(1), goalA(2), goalA(3))
        % Transfer node from sub to index.
        nodeInd = matrixToIndex(node);
        
        % Find the neighbors of new node in visited and define its cost vector.
        neighbors = findNgb(map, node);
        
        % Check neighbors not in the visited.
        for i = 1:size(neighbors,1)
            % If the neighbors are visited skip this iteration.
            if visited(neighbors(i,1), neighbors(i,2), neighbors(i,3))
                continue;
            end
            nghbInd = matrixToIndex(neighbors(i,:));
            stepCost = sum(abs((neighbors(i,:)-node).*[paceLength(1),paceLength(2),paceLength(3)]));
            % If the non-visited neighbors are not in the reachable, add them
            % and update the cost.
            if ~exploredSet(neighbors(i, 1), neighbors(i, 2), neighbors(i, 3))
                reachable = [reachable; neighbors(i, :)];
                exploredSet(neighbors(i, 1), neighbors(i, 2), neighbors(i, 3)) = 1;
                parent(nghbInd) = nodeInd;
                cost(nghbInd) = stepCost + cost(nodeInd);
                % If they are already in the reachable, update the cost to be the minimum.
            elseif stepCost + cost(nodeInd) < cost(nghbInd)
                cost(nghbInd) =  stepCost + cost(nodeInd);
                parent(nghbInd) = nodeInd;
            end
        end
        
        % Use matrix finding min to get the min cost point
        [~, minNum] = min(cost(matrixToIndex(reachable)) + hFun(reachable));
        
        
        % Extract minCost from reachable. Add node and minCost to visited and
        % update current node.
        node = reachable(minNum,:);
        
        exploredSet(node(1), node(2), node(3)) = 0;
        reachable(minNum, :) = [];
        % Update visited.
        visited(node(1), node(2), node(3)) = 1;
    end
    
else
    while any(size(reachable,1))&& ~visited(goalA(1), goalA(2), goalA(3))
        % Transfer node from sub to index.
        nodeInd = matrixToIndex(node);
        
        % Find the neighbors of new node in visited and define its cost vector.
        neighbors = findNgb(map, node);
        
        % Check neighbors not in the visited.
        for i = 1:size(neighbors,1)
            % If the neighbors are visited skip this iteration.
            if visited(neighbors(i,1), neighbors(i,2), neighbors(i,3))
                continue;
            end
            nghbInd = matrixToIndex(neighbors(i,:));
            % If the non-visited neighbors are not in the reachable, add them
            % and update the cost.
            stepCost = sum(abs((neighbors(i,:)-node).*[paceLength(1),paceLength(2),paceLength(3)]));
            
            if ~exploredSet(neighbors(i, 1), neighbors(i, 2), neighbors(i, 3))
                reachable = [reachable; neighbors(i, :)];
                exploredSet(neighbors(i, 1), neighbors(i, 2), neighbors(i, 3)) = 1;
                parent(nghbInd) = nodeInd;
                cost(nghbInd) = stepCost + cost(nodeInd);
                % If they are already in the reachable, update the cost to be the minimum.
            elseif stepCost + cost(nodeInd) < cost(nghbInd)
                cost(nghbInd) =  stepCost + cost(nodeInd);
                parent(nghbInd) = nodeInd;
            end
        end
        
        % Use matrix finding min to get the min cost point
        [~, minNum] = min(cost(matrixToIndex(reachable)));
        % Extract minCost from reachable. Add node and minCost to visited and
        % update current node.
        node = reachable(minNum,:);
        
        exploredSet(node(1), node(2), node(3)) = 0;
        reachable(minNum, :) = [];
        % Update visited.
        visited(node(1), node(2), node(3)) = 1;
    end
end









%% LOAD DATA
index = 1;
goalG = matrixToIndex(goalA);
paInd(1) = goalG;
startG = matrixToIndex(startA);

while ~(goalG == startG)
    index = index + 1;
    goalG = parent(goalG);
    paInd(index) = goalG;
end
[pathX pathY pathZ]= indexToMatrix(paInd);
pathAry = flipud([pathX(:),pathY(:),pathZ(:)]);
path = [ary2re(pathAry); goal];
path(1,:) = start;
num_expanded = sum(visited(:));
end