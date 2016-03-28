function neigborData = findNgb(map,node)
% Find all the possible neighbors for a position.

    neigborData = [node(1)+1, node(2), node(3);
    node(1)-1, node(2), node(3);
    node(1), node(2)+1, node(3);
    node(1), node(2)-1, node(3);
    node(1), node(2), node(3)+1;
    node(1), node(2), node(3)-1];

temp = [];
mapdataSample = map.map;
for i = 1:size(neigborData,1)
    if (neigborData(i,1) >= 1) && (neigborData(i,2) >= 1) && (neigborData(i,3) >= 1) && (neigborData(i,1) <= map.colums) && (neigborData(i,3) <= map.depthes) && (neigborData(i,2) <= map.rows)
        if mapdataSample(neigborData(i,1),neigborData(i,2),neigborData(i,3))~=1
            temp = [temp;neigborData(i,:)];
        end
        
    end
end
neigborData = temp;
end