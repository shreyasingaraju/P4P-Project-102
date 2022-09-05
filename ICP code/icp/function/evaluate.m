function [RMS, distance] = evaluate (ref, data)


    diff = zeros(1,length(data));
    distance = 0;
    for i = 1:length(data)
        
        x1 = data(1,i);
        y1 = data(2,i);
        min = 100;
        for j = 1 : length(ref)
            x2 = ref(1,j);
            y2 = ref(2,j);
            
            dx = x2 - x1;
            dy = y2 - y1;
            dd = sqrt(dx^2 + dy^2);
            if dd < min
                min = dd;
            end
        end
        
        
        if i ~= length(data)
            x3 = data(1,i+1);
            y3 = data(2,i+1);
            
            dx = x3 - x1;
            dy = y3 - y1;
            dd = sqrt(dx^2 + dy^2);
            distance = distance + dd;
        end
        
        diff(i) = min^2;
        
    end
    
    RMS = sqrt(sum(diff)/length(data));
end