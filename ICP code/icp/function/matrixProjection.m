function [result] = matrixProjection(D3_matrix,startPoint)
    
% This function is projecting the 3D position into 2D.
% Input: 
%       D3_matrix: Is the 3D matrix that contain the spatial information of
%       the trajectory
%       startPoint: Where the projection start with. (usually 1, which
%       project the whole matrix from the beginning)
% Output: 
%       result: The 2D position information after the prjection.
% Author Huyue Wang

    len = size(D3_matrix);
    len = len(2);
    
     temp = zeros(4,len-1);
    %-=---- work out displacement
    for k = startPoint:(len-1)
        x1 = D3_matrix(1,k);
        y1 = D3_matrix(2,k);
        z1 = D3_matrix(3,k);

        x2 = D3_matrix(1,k+1);        
        y2 = D3_matrix(2,k+1);
        z2 = D3_matrix(3,k+1);

        dx = x2 - x1;
        dy = y2 - y1;
        dz = z2 - z1;

        dd = sqrt(dx^2 + dy^2 + dz^2);
        temp(1,k) = dx;
        temp(2,k) = dy;
        temp(3,k) = dz;
        temp(4,k) = dd;

    end
    %------------------------------

  
    for i = startPoint:len-1
        
        

        
        x1 = D3_matrix(1,i);
        y1 = D3_matrix(2,i);
        z1 = D3_matrix(3,i);
        
        x2 = D3_matrix(1,i+1);        
        y2 = D3_matrix(2,i+1);
        z2 = D3_matrix(3,i+1);
        
        D3_matrix(3,i+1) = z1;
        s = temp(4,i)/sqrt(temp(1,i)^2 + temp(2,i)^2);
        
        D3_matrix(1,i+1) = x1 + s * temp(1,i);
        D3_matrix(2,i+1) = y1 + s * temp(2,i);
        for j = (i+1):len-1
             D3_matrix(1,j+1) =  D3_matrix(1,j) + temp(1,j);
             D3_matrix(2,j+1) =  D3_matrix(2,j) + temp(2,j);
             D3_matrix(3,j+1) =  D3_matrix(3,j) + temp(3,j);
        end
        
    end
    result = D3_matrix;

end