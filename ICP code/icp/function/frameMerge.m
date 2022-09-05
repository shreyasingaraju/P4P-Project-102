function [mergedFrame] = frameMerge (frame,mergedF)
%      Merging the frame
% input 
%       frame: #. of the frame, the point that owned by the same frame will
%       labled with the same serial number
%       mergedF: number of frame that wants to merged
% output
%       mergedFrame: #.frame after merged, have a same sturcture with the
%       input 'frame'
% Author: Huyue

    m_size = length(frame);
    num_Switch = 0;
    m_compare = frame(1);
    m_write = frame(1);
    for i = 1: m_size - 1
        if m_compare ~= frame(i+1)
            num_Switch = num_Switch + 1;
            m_compare = m_compare + 1;
        end
        frame(i) = m_write;
        if num_Switch == mergedF
            m_write = m_write +1;
            num_Switch = 0;
        end
        if m_compare > 255
            m_compare = 0;
        end
        if m_write > 255
            m_write = 1;
        end
    end
frame(m_size) = frame(m_size - 1);
mergedFrame = frame;
end