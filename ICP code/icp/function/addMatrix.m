function [output] = addMatrix(M1,M2)
% This function is append the M2 to the M1
% Huyue Wang

S1 = size(M1);
S2 = size(M2);



if S1(2) == 0
    
    ST = S1(2) + S2(2);
    output = zeros(S1(1),ST);
    output(1:S2(1),1:S1(2)) = M1;
    output(1:S2(1),1:S2(2)) = M2;
else
    ST = S1(2) + S2(2);
    output = zeros(S1(1),ST);
    output(1:S2(1),1:S1(2)) = M1;
    
    output(1:S2(1),(S1(2)+1):ST) = M2;
end


end