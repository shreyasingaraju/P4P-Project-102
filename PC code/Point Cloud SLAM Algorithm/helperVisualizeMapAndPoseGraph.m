function helperVisualizeMapAndPoseGraph(omap, pGraph, ax)
% This helper function is useful for visualizing the built occupancy map 3D
% (omap) and pose graph (pGraph). The plot view is tuned for this example.

%   Copyright 2019 The MathWorks, Inc.

disp("Using helperVisualizeMapAndPoseGraph.m");

show(omap,'Parent',ax);
hold on;
pGraph.show('Parent',ax,"IDs","off");
xlim([-10 10]);
ylim([-10 10]);
zlim([-10 10]);
view([10,10]);
drawnow
hold off;
grid on;
end

