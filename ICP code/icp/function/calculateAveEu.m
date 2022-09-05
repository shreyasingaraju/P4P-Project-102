function [aveEu] = calculateAveEu (traj)
% This function is calculating the average Euclidean distance between the
% location stored in the traj.
% The input is a 3D matrix contain the 3D position information
% The output is the average Euclidean distance.
% Author: Huyue Wang
eu_sum = 0;
    for currentP = 1: length(traj)-1
        cur = traj(:,currentP);
        nx = traj(:,currentP+1);

        x_nx = sqrt((cur(1,:)-nx(1,:))^2);
        y_nx = sqrt((cur(2,:)-nx(2,:))^2);
        z_nx = sqrt((cur(3,:)-nx(3,:))^2);

        eu_pre = sqrt(x_nx^2 + y_nx^2 + z_nx^2);
        eu_sum = eu_sum + eu_pre;
    end
    
    aveEu = eu_sum/length(traj);

end