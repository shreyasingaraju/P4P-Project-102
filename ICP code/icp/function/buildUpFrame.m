function [sortedFrame] = buildUpFrame (poinFrame,frame)
    % remove the frame in pointFrame from the frame
    % input:
    %       pointFrame: 
    %           list that contain the fram that need to be
    %           filtered
    %       frame:
    %           list that contain all the frame
    % output:
    %       sortedFrame:
    %          sortedFram, which is frame - pointFrame
    % Author: Huyue
    ms =  length(frame);
    ps = length(poinFrame);
    sortedFrame = zeros(1,ms-ps);
    flage = 0;
    pointCount = 1;
    for i = 1:ms
        for j = 1:ps
            if i == poinFrame(j)
                flage = 1;
            end
        end
        if flage == 0
            sortedFrame(pointCount) = frame(i);
            pointCount = pointCount +1;
        else
            flage = 0;
        end
    end
    
end