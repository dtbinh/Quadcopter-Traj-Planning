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
dimensions = map.dimensions;
re2ary = map.re2ary;
ary2re = map.ary2re;
nghbFind = map.nghbFind;


% Define a function to calculate the cost and heuristic distance between 
% points A & B.
costCal = @(nghbAry, nodeCurrent) sqrt(sum(bsxfun(@power, bsxfun(@minus,ary2re(nghbAry),ary2re(nodeCurrent)),2),2));
hrCost = @(nghbAry) sqrt(sum(bsxfun(@power, bsxfun(@minus, ary2re(nghbAry), goal), 2.0), 2));
% Transfer goal and start to our grid system
startA = re2ary(start);
goalA = re2ary(goal);
node = startA;
% Define function transferring sub to index and index to sub.
s2i = @ (sub) sub(:,1)+ (sub(:,2)-1) * colums + (sub(:,3)-1) * colums * rows;
i2s = @ (index) ind2sub(unitL, index);

% Initialize three space matrice: visited, reachable, parent representing
% the cost from start to each discrete point in the map.
visited = zeros(unitL);
reachable = nghbFind(map,node);
reachFlag = zeros(unitL);
parent = zeros(dimensions, 1);

% Define the cost matrix the initial value is infinite.
cost = inf(dimensions, 1);
cost(s2i(node)) = 0;
cost(s2i(reachable)) = costCal(reachable,node);

%% FIND THE GOAL
while any(size(reachable,1))&& ~visited(goalA(1), goalA(2), goalA(3))
    % Transfer node from sub to index.
    nodeInd = s2i(node);
    
    % Find the neighbors of new node in visited and define its cost vector.
    neighbors = nghbFind(map, node);
    costNghb = costCal(neighbors,node);
    
    % Check neighbors not in the visited.
    for i = 1:size(neighbors,1)
        % If the neighbors are visited skip this iteration.
    	if visited(neighbors(i,1), neighbors(i,2), neighbors(i,3))
			continue;
        end
        nghbInd = s2i(neighbors(i,:));
        % If the non-visited neighbors are not in the reachable, add them 
        % and update the cost.
        if ~reachFlag(neighbors(i, 1), neighbors(i, 2), neighbors(i, 3))%~ismember(neighbors(i,:),reachable,'rows')
            reachable = [reachable; neighbors(i, :)];
            reachFlag(neighbors(i, 1), neighbors(i, 2), neighbors(i, 3)) = 1;
            parent(nghbInd) = nodeInd;
            cost(nghbInd) = costNghb(i) + cost(nodeInd);
        % If they are already in the reachable, update the cost to be the minimum.
        elseif costNghb(i) + cost(nodeInd) < cost(nghbInd)
                cost(nghbInd) =  costNghb(i) + cost(nodeInd);
                parent(nghbInd) = nodeInd;
        end
    end
    
    % Use matrix finding min to get the min cost point
    if astar
    [~, minNum] = min(cost(s2i(reachable)) + astar * hrCost(reachable));
    else
    [~, minNum] = min(cost(s2i(reachable)));
    end
    
    % Extract minCost from reachable. Add node and minCost to visited and 
    % update current node.
    node = reachable(minNum,:);
    reachable(minNum, :) = [];
    reachFlag(node(1), node(2), node(3)) = 0;
    
    % Update visited.
    visited(node(1), node(2), node(3)) = 1;       
end

%% LOAD DATA
paNum = 1;
paPoi = s2i(goalA);
paInd(1) = paPoi;
paSt = s2i(startA);
    
while ~(paPoi == paSt)
    paNum = paNum + 1;
    paPoi = parent(paPoi);
    paInd(paNum) = paPoi;
end
[pathX pathY pathZ]= i2s(paInd);
pathAry = flipud([pathX(:),pathY(:),pathZ(:)]);
path = [ary2re(pathAry); goal];
path(1,:) = start;
num_expanded = sum(visited(:));
end