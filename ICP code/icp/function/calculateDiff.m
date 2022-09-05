function [indicator] = calculateDiff(currentP,traj,ave)

% This function is calculating the Euclidean distance  between the current
% position and next posistion in the distance, and then indicate wether the
% difference larger than the average. 
%Input:
%     currentP: indicate the current position.
%     traj: matrix that contain all the position information of the
%     trjaectory
%     ave: The average Euclidean disatance among the whole trajectory.
%Output: 
%     indicator: shows wether the difference larger than the average or
%     not.
    cur = traj(:,currentP);
    nx = traj(:,currentP+1);
    pre = traj(:,currentP-1);
    
%     x_pre = sqrt((cur(1,:)-pre(1,:))^2);
%     y_pre = sqrt((cur(2,:)-pre(2,:))^2);
%     z_pre = sqrt((cur(3,:)-pre(3,:))^2);
%     
%     eu_pre = sqrt(x_pre^2 + y_pre^2 + z_pre^2);
    
    x_nx = sqrt((cur(1,:)-nx(1,:))^2);
    y_nx = sqrt((cur(2,:)-nx(2,:))^2);
    z_nx = sqrt((cur(3,:)-nx(3,:))^2);
    
    eu_nx = sqrt(x_nx^2 + y_nx^2 + z_nx^2);
    

   if (eu_nx > ave * 1.3 )
       indicator = 1; 
%    elseif (eu_pre > ave * 1.2 )
%        indicator = 2; 
   elseif (eu_nx < ave * 0.6 )
       indicator = 1; 
%     elseif (eu_pre < ave * 0.6 )
%        indicator = 2; 
   else
       indicator = 0;
   end
    
end