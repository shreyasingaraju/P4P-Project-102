function [dataOut,frameOut] = nearestNeighborsFilter (rawData,K,threshold )
% Nearest Neighbors Filter that filters out outlier data
% For every point in the rawdata matrix, the euclidean distance  from the selected
% point to every other point  is calculated. The number of points that
% has a euclidean distance less than the threshold  value will be
% summed. And any point with the summed value less than the K value will be
% filitered.

%input: 
%       rawData: the matrix that contain x y z value 
%       K: point-number threshold
%       threshold: distance threshold
%output:
%       dataOut: the matrix that contain filted data
%       frameOut: frame corresponding to the dataOut
% Author: Huyue
dataLength = length(rawData);
euDistance = zeros(2,dataLength);
neighborsSum = 0;
newData = [];
newFrame = [];
for i = 1:dataLength
    for j = 1:dataLength
        if j ~= i
            x = rawData(1,i)- rawData(1,j);
            y = rawData(2,i)- rawData(2,j);
            z = rawData(3,i)- rawData(3,j);
            euc = sqrt(x^2 + y^2 + z^2);
            euDistance(1,i) = euc;
            if euc < threshold 
                 euDistance(2,i) =  euDistance(2,i)+ 1;
            end
        end
    end
    neighborsSum = neighborsSum + euDistance(2,i);
end
if K == 'A'

    K = neighborsSum/length(rawData)*0.1;
    
end
for i = 1:dataLength

     
    if euDistance(2,i) >= K
        newData = [newData,;rawData(1,i),rawData(2,i),rawData(3,i)];
    else
        newFrame = [newFrame;i];
    end
end

dataOut = pagetranspose(newData);
frameOut = pagetranspose(newFrame);
end