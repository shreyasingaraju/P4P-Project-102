function [D2Angle] = calculateD2Measurement(D2Matrix)
% This funcion is translating the 2D position information format from the (x,y)
% to (r,theta). Where r is the radius and theata is the angle.
% Author Huyue Wang
    s = size(D2Matrix);
    D2Angle = zeros(2,s(2));
    
    for i = 1:s(2)
        x = D2Matrix(1,i);
        y = D2Matrix(2,i);
        L = sqrt(x^2 + y^2);
        theata = atan( abs(x)/abs(y));
        theata = theata * 180/pi;
        
        if x >0 && y > 0
        %---------------1    
        elseif x > 0 && y < 0
            %---------------------2
            theata = -abs(theata);
        elseif x < 0 && y < 0
            %----------------------3
            theata = -(180 - abs(theata));
        elseif x < 0 && y > 0
            %------------------------4
            theata = 180 - abs(theata);
        end
        D2Angle(1,i) = L;
        D2Angle(2,i) = theata;
    end

end