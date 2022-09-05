function [displacement] = calculateDisplacement(D2_matrix)
% This function is calculating the 2D dispalcement information
% Input:
%       D2_matrix: This is 2D matrix that conatin 2D position information
%       of the trajectory.
% Output: 
%      The output is the 4xX matrix 2D displacement information. The first
%      row representing  the movement along x direction, the second row
%      representing  the movement along y direction, third row is the rotation
%      from the previous  position to the current position, and the last row
%      is the overall orientation of the object.
% Author Huyue Wang

    displacement =  zeros(4,length(D2_matrix)-1);
    local = zeros(1,length(D2_matrix)-1);
    glob = zeros(1,length(D2_matrix)-1);
    
    glob(1) = 90;
    local(1) = 90; 
    xT = zeros(1,length(D2_matrix)-1);
    yT = zeros(1,length(D2_matrix)-1);
    
    xT(1) = D2_matrix(1,2);
    yT(1) = D2_matrix(2,2);
    
    for i = 2:length(D2_matrix)-1
        
        a = i + 5;
        x1 = D2_matrix(1,i);
        y1 = D2_matrix(2,i);
        
        x2 = D2_matrix(1,i+1);
        y2 = D2_matrix(2,i+1);
        
        dx = x2 - x1;
        dy = y2 - y1;
        
        xT(i) = dx;
        yT(i) = dy;
        
        temp = zeros(2,5);
        if(a >= length(D2_matrix))
            glob(i) = glob(i-1);
        else
            %-------------------------- working direction
            for j = i:a-1
                %---------------------- working the tendency
                xa = D2_matrix(1,j+1);
                ya = D2_matrix(2,j+1);

                temp(1,j-1) = xa - x1;
                temp(2,j-1) = ya - y1; 
                
            end
            
            dxT = sum(temp(1,:));
            dyT = sum(temp(2,:));
            
            if( abs(dxT) > abs(dyT ))
                %-------------------- x movement
                if dxT > 0
                    %----- right
                    glob(i) = 0;
                else
                    %----- left
                    glob(i) = 180;
                end
            else
                %-------------------- y movement
                if dyT > 0
                    %----- forward
                    glob(i) = 90;
                else
                    %----- backward
                    glob(i) = -90;
                end
            end
            
        end
       
    end
    
    for k = 2: length(glob)
        
        local(k) = glob(k) - glob(k-1);
        
    end
    
    pre = 0;
    for k = 1: length(glob)
        
       %------------------------- corrdinate trans
        cur = glob(k);
        
        if(abs(pre) == 90)
            if pre > 0
                temp = xT(k);
                xT(k) = yT(k);
                yT(k) = -temp;
            else
                temp = xT(k);
                xT(k) = -yT(k);
                yT(k) = temp;
            end
        elseif(abs(pre) == 180)

            xT(k) = -xT(k);
            yT(k) = -yT(k);

        elseif(abs(pre) == 270)
            if pre < 0
                temp = xT(k);
                xT(k) = yT(k);
                yT(k) = -temp;
            else
                temp = xT(k);
                xT(k) = -yT(k);
                yT(k) = temp;
            end
        end
        pre = cur;
        
%-------------------------------------
       
        
    end
%     for j = 1:length(local)
%         local(j) = local(j) * (pi/180);
%         glob(j) = glob(j) * (pi/180);
%     end
    displacement(1,:) = xT;
    displacement(2,:) = yT;
    displacement(3,:) = local;
    displacement(4,:) = glob;
end