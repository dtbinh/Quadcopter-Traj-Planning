function plot_map(map)
% plotting example loaded from other function call

%% INITIALIZE
% getting the information from the previous map
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;


%% Using the built-in plot function to plot the map
for i = 1: blockNum
    vertexMat = find_vertex(map,i);
    % Plot the block.
    patch('Vertices',vertexMat,'Faces',[1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8],'FaceColor',block{i}.color)
end

% Load data with margin into vertexMat.
mapp.blockNum = blockNum;
for i = 1: blockNum
    mapp.block{i}.lowerleft = block{i}.lowerleft - margin; 
    mapp.block{i}.upperright = block{i}.upperright + margin; 
    
end

for i = 1:blockNum
    vertexMat = find_vertex(mapp,i);
    % Plot the block margin.
    patch('Vertices',vertexMat,'Faces',[1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8],'FaceColor',block{i}.color,'FaceAlpha',0.5,'EdgeColor','none')
end


% Open figure 1.
figure();
axis equal
axis([ boundary.lowerleft(1) boundary.upperright(1),boundary.lowerleft(2) boundary.upperright(2),boundary.lowerleft(3) boundary.upperright(3)]);
grid on

% Title of figure.
title('MEAM 620 Phrase 3');

% Label the axes.
xlabel('X')
ylabel('Y')
zlabel('Z')
end

