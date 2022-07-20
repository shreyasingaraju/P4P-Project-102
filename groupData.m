clear;
clc;
close all;

x = [
    -0.03320312	-0.2109375	-0.06445312	0.00390625	0.00390625...
    -0.04882812	-0.22070312	-0.03320312	-0.22460938	0.00585938...
    0.05273438	-0.2109375	-0.04882812	-0.1328125	0.00390625...
    -0.27929688	-0.03320312	-0.06445312	-0.4453125	0.00390625...
    -0.22070312	-0.04882812	0.00390625	0.00390625	-0.12890625
    ];

y = [
    0.03710938	0.71875	0.10546875	0.13476562	0.125... 
    0.0546875	0.65820312	0.04882812	0.532356	0.171875...
    0.41796875	0.71679688	0.0703125	0.18164062	0.0859375...
    0.542567	0.05859375	0.08007812	0.36132812	0.12890625...
    0.66796875	0.08789062	0.08789062	0.08007812	0.12890625
    ];

z = [
    0.080078	0.011719	0.142578	-0.04297	-0.06445...
    0.119141	0.111328	0.074219	-0.39844	-0.07617...
    -0.0332	0.072266	0.111328	-0.41211	-0.03906...
    -0.42578	0.066406	0.15625	-0.48242	-0.05469...
    0.035156	0.097656	-0.03516	-0.04688	-0.04102 
];

% x = [-0.21 -0.22 -0.21 -0.22];
% y = [0.72 0.66 0.72 0.67];
% z = [0.07 0.11 0.01 0.04];
% 


% count = 1;
% distanceSquared = 0;
% 
% for i= 1:length(x)
%     coord1 = [xyz(1, i, 1), xyz(1, i, 2), xyz(1, i, 3)];
% 
%     for j = 1:length(x)
%         coord2 = [xyz(1, j, 1), xyz(1, j, 2), xyz(1, j, 3)];
%         
%         % Compare distance
%         if (~(isequal(coord1, coord2)))
%             distanceSquared = (coord1(1) - coord2(1))^2 + ...
%             (coord1(2) - coord2(2))^2 + (coord1(3) - coord2(3))^2;
% 
%             distances(count) = distanceSquared;
%             % If within distance threshold, consider the point a close
%             % neighbour
%             if (distanceSquared <= 0.15 && distanceSquared ~= 0)
%                 if (coord1 ~= [0 0 0] & coord2 ~= [0 0 0])
%                     seenPoints(count, :) = {[i, j]};
%                     groupX(count) = abs((coord1(1) + coord2(1))/2);
%                     count = count + 1;
%                 end 
%             end
%         end
%     end
% end
% 
% count2 = 1;
% for i = 1:length(seenPoints)
%     unique1 = cell2mat(seenPoints(i, :))
% 
%     for j = 1:length(seenPoints)
%         unique2 = cell2mat(seenPoints(j, :))
%         
%         if (~(isequal(unique1, unique2)) && ~(unique1(1) == unique2(2) && unique1(2) == unique2(1)))
%             groupedCoords(count2, :) = {unique1, unique2};
%             count2 = count2 + 1;
%         elseif (unique1(1) == unique2(2) && unique1(2) == unique2(1))
%             fprintf("works: %d %d\n", unique1, unique2);
%         end
%     end
% end
% 
% scatter3(x, y, z);

x = transpose(x);
y = transpose(y);
z = transpose(z);

data(:, 1) = x;
data(:, 2) = y;
data(:, 3) = z;

% sampleData = rand(1000, 5); % 1000 samples with 3 features
[idx, C] = kmeans(data, 4); % 3 number of classes
plot3(C(:,1),C(:,2),C(:,3),'rx','MarkerSize',15,'LineWidth',3)
grid
hold on
scatter3(data(:,1), data(:,2), data(:,3), 15, idx, 'filled');
hold on

xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
