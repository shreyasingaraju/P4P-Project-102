function [matrix]  = buildUpMatrix(x,y,z,frame)
    sz = length(z) - length(frame);
    new_matrix = zeros(3,sz);
    flage = 0;
    frame_indicator = 1;
    for i = 1:length(x)
        for j = 1:length(frame)
            if i == frame(j)
                flage = 1;
            end
        end
        if flage ~= 1
            new_matrix(1,frame_indicator) = x(i);
            new_matrix(2,frame_indicator) = y(i);
            new_matrix(3,frame_indicator) = z(i);
            frame_indicator = frame_indicator +1;
        else
            flage = 0;
        end
    end
    matrix = new_matrix;
end