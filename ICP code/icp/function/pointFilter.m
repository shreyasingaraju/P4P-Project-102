function [zero_frame] = pointFilter (x,y,z,MAX_RANGE,MIN_RANGE,flage)
% Remove all point with z value less than 0.05 and larger than maximum ambiguous range ( can not be processed by
% iSAM2.).  Also remove the points that out of the MAX_RANGE and/or within
% the MIN_RANGE
%input:
%      x,y,z: the x y z value for each point
%      MAX_RANGE: Maximim range of the data, any poiont futher than it will be filtered
%      MIN_RANGE: Minimum range of the data, any point within the distance will be filtered
%      flage: indicate 3D or 2D condition
%output: 
%      zero_frame: the frames that be filtered
% Author: Huyue

if flage == 1
        filterZ = 0;

        for i = 1: length(z)
%             if(abs(z(i)) < 0.005 )
%                  filterZ = filterZ + 1;
%             elseif(abs(y(i)) < 0.005 )
%                  filterZ = filterZ + 1;
%             elseif(abs(x(i)) < 0.005 )
%                  filterZ = filterZ + 1;
            if(sqrt((x(i)^2)+(y(i)^2)+(z(i)^2)) > MAX_RANGE )
                filterZ = filterZ + 1;
            elseif(sqrt((x(i)^2)+(y(i)^2)+(z(i)^2)) < MIN_RANGE )
                filterZ = filterZ + 1;
            end

        end
        % matrix of the point that not fit the cratira 
        zero_frame = zeros(1,filterZ);
        Z_flag = 1;
        for i = 1: length(z)
%              if(abs(z(i)) < 0.005)
%                  zero_frame(Z_flag) = i;
%                  Z_flag = Z_flag+1;
%              elseif(abs(x(i)) < 0.005)
%                  zero_frame(Z_flag) = i;
%                  Z_flag = Z_flag+1;
%              elseif(abs(y(i)) < 0.005)
%                  zero_frame(Z_flag) = i;
%                  Z_flag = Z_flag+1;

             if(sqrt((x(i)^2)+(y(i)^2)+(z(i)^2)) > MAX_RANGE )
                zero_frame(Z_flag) = i;
                Z_flag = Z_flag+1;



            elseif(sqrt((x(i)^2)+(y(i)^2)+(z(i)^2)) < MIN_RANGE )
                zero_frame(Z_flag) = i;
                Z_flag = Z_flag+1;


            end
        end
elseif (flage == 2)
     filterZ = 0;
    for i = 1: length(z)
         if(z(i) < -0.5 )
            filterZ = filterZ + 1;
         elseif(z(i)>0.5)
             filterZ = filterZ + 1;
         elseif(y(i)<0.1)
             filterZ = filterZ + 1;
         elseif(sqrt((x(i)^2)+(y(i)^2)+(z(i)^2)) < 0.5 )
             filterZ = filterZ + 1;
         elseif(sqrt((x(i)^2)+(y(i)^2)+(z(i)^2)) > 1.5 )
             filterZ = filterZ + 1;
         end
    end
     zero_frame = zeros(1,filterZ);
     Z_flag = 1;
     for i = 1: length(z)
        if(z(i) < -0.5 )
            zero_frame(Z_flag) = i;
            Z_flag = Z_flag+1;
        elseif(z(i)>0.5)
            zero_frame(Z_flag) = i;
            Z_flag = Z_flag+1;
        elseif(y(i)<0.1)
            zero_frame(Z_flag) = i;
            Z_flag = Z_flag+1;
        elseif(sqrt((x(i)^2)+(y(i)^2)+(z(i)^2)) < 0.5 )
            zero_frame(Z_flag) = i;
            Z_flag = Z_flag+1;
        elseif(sqrt((x(i)^2)+(y(i)^2)+(z(i)^2)) > 1.5 )
            zero_frame(Z_flag) = i;
            Z_flag = Z_flag+1;
        end
     end
end

end