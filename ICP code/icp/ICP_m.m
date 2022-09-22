%
%The input is the point cloud data from the mmW Radar by the UART reader. 
%The input is a 4*X matrix contains 4 sets of variable, which named as: frame, x,y, and z.
%e.g.
% frame: 1 1 1 2 2 2 3 3 3 3 ...  
%     x: 1 2 1 1 1 2 2 1 1 2 ...
%     y: 1 1 2 2 3 3 3 2 2 2 ...
%     z: 1 2 2 2 2 2 2 1 1 1 ...

%The frame indicates the number of frame that the x,y,and z value in the same column belongs
%to, and the x,y,z indicates 3D spatial information.
 
% For instance, the example of input data shown above illustrates 3 sensed 
% points that belong to frame 1 followed by 3 points from frame 2 and 4 points from frame 3.

clc;
clear;
close all;
addpath('../../ICP code/icp/function');

format long g
% The input filed
load('../../Sensor data/ICP data/Data (long room)/LongRoomSlow_1.mat')



% ICP setting for ICP algorithm 1, not needed for ICP algorithm 2. 
ICP_Func = 2;
%Maximim range of the data, any poiont futher than it will be filtered
MAX_RANGE =9;
%Minimum range of the data, any point within the distance will be filtered
MIN_RANGE = 0.75;
% Number of frame that be merged
mergedF = 10;
% K number of the nearest neighbors algorithm. Points that have the number
% of neighbor less than 10% of the average will be filtered when K = 'A;
K = 'A';
% range of k nearest neighbors algorithm   5
threshold  = 5;
% when 1 ICP algorithm 1 will be selected
% when 2 ICP algorithm 2 will be selected
ICPSELECT = 2;


%------frame sort----

% Sorting the frame, make sure the sequence of the frame number is
% continuous,
write = 1;
for i = 1:length(frame)-1
    if frame(i) == frame(i+1)
        frame(i) = write;
    else
        frame(i) = write;
        write = write +1;
        if write == 256
            write = 1;
        end
       
    end
end
frame(i+1) = write;

% Used for debugging
ref_frame = frame;
%---------------------frame merge
% merging the frame by changing the indicated frame value.
new_frame = frameMerge(frame,mergedF);

x = x(1:length(frame));
y = y(1:length(frame));
z = z(1:length(frame));

new_matrix(1,:) = x;
new_matrix(2,:) = y;
new_matrix(3,:) = z;

ref_matrix = new_matrix;
%---------Filtering---------

%      zero_frame: the frames that be filtered
[zero_frame] = pointFilter (x,y,z,MAX_RANGE,MIN_RANGE,1);

%--------------------------Build up new frame
%sorting the frame 
new_frame = buildUpFrame (zero_frame,new_frame);
%--------------------------Build up matrix
% producing the matrix contain the x y and z value corresponding to the sorted frame 
new_matrix = buildUpMatrix(x,y,z,zero_frame);
%-------------Frame Counting---------------
% 
f_length = length(new_frame);
frameC = zeros(1,new_frame(f_length));
for k = 1:length(new_frame)
    
    frameC(new_frame(k)) = frameC(new_frame(k))+1;
%     if k<=255
%         frameC(new_frame(k)) = frameC(new_frame(k))+1;
%     else
%         frameC(new_frame(k)+255) = frameC(new_frame(k)+255)+1;
%     end
end

%--------------------------------------
frameNum = length(frameC);
T_matrix = zeros(3,frameNum);
R_matrix = zeros(3,frameNum*3);
%------------------------------------
%set up current point
atFrame = 1;
totalPoint = frameC(1);
prePoint = totalPoint;

% Build up first frame


model = zeros(3,totalPoint);
model(1,:) = new_matrix(1,1:totalPoint);
model(2,:) = new_matrix(2,1:totalPoint);
model(3,:) = new_matrix(3,1:totalPoint);
%---------------------------KN-Filter for modle --------------

  [model,Filter_frame] = nearestNeighborsFilter (model,K,threshold );


% Build up second frame
totalPoint = totalPoint+frameC(2);

data = zeros(3,(totalPoint-prePoint));
data(1,:) = new_matrix(1,prePoint+1:totalPoint);
data(2,:) = new_matrix(2,prePoint+1:totalPoint);
data(3,:) = new_matrix(3,prePoint+1:totalPoint);


%---------------------------KN-Filter for data--------------

  [data,Filter_frame] = nearestNeighborsFilter (data,K,threshold );


prePoint = totalPoint;



if ICPSELECT == 1
    [RotMat,TransVec,dataOut]=icp(model,data,10,500,ICP_Func);
elseif ICPSELECT == 2
     [RotMat,TransVec] = icp2(model, data, 1000,'Matching', 'kDtree','Minimize','point');
     dataOut = RotMat * data + repmat(TransVec, 1, size(data,2));

end
T_matrix(:,1) = TransVec;
R_matrix(:,1:3) = RotMat;
atFrame = atFrame+1;


% visualizing
% figure(9)
% subplot(1,2,1)
% title('Original data points (blue) and model points (red)')
% plot3(model(1,:),model(2,:),model(3,:),'r.'), axis equal
% 
% subplot(1,2,2)
% plot3(data(1,:),data(2,:),data(3,:),'b.'), axis equal
% 
% 
% figure(8)
% subplot(1,2,1)
% 
% plot3(model(1,:),model(2,:),model(3,:),'r.',data(1,:),data(2,:),data(3,:),'b.'), axis equal
% title('Original data points (blue) and model points (red),Transformed data points (green)')
% 
% xlabel('x')
% ylabel('y')
% zlabel('z')
% 
% subplot(1,2,2)
% 
% plot3(model(1,:),model(2,:),model(3,:),'r.',dataOut(1,:),dataOut(2,:),dataOut(3,:),'g.'), axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
 
%loop
j = 3;
while j < length(frameC)
    
     model = data;
     totalPoint = totalPoint+frameC(j);
    

        data = zeros(3,totalPoint-prePoint);
        data(1,:) = new_matrix(1,prePoint+1:totalPoint);
        data(2,:) = new_matrix(2,prePoint+1:totalPoint);
        data(3,:) = new_matrix(3,prePoint+1:totalPoint);
        
%---------------------------KN-Filter--------------

  [data,Filter_frame] = nearestNeighborsFilter (data,K,threshold );

  
if ICPSELECT == 1
    [RotMat,TransVec,dataOut]=icp(model,data,10,500,ICP_Func);
elseif ICPSELECT == 2
     [RotMat,TransVec] = icp2(model, data, 1000,'Matching', 'kDtree','Minimize','point');
     dataOut = RotMat * data + repmat(TransVec, 1, size(data,2));
end
    T_matrix(:,atFrame) = TransVec;
    R_matrix(:,(atFrame-1)*3+1:atFrame*3) = RotMat;
    atFrame = atFrame + 1;
    nextPose = j;
    
%     figure(9)
%     
%     subplot(1,2,1)
%     title('Original data points (blue) and model points (red)')
%     plot3(model(1,:),model(2,:),model(3,:),'r.'), axis equal
% 
%     subplot(1,2,2)
%     plot3(data(1,:),data(2,:),data(3,:),'b.'), axis equal
% 
% 
%     figure(8)
%     
%     subplot(1,2,1)
%     plot3(model(1,:),model(2,:),model(3,:),'r.',data(1,:),data(2,:),data(3,:),'b.'), axis equal
%     title('Original data points (blue) and model points (red),Transformed data points (green)')
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
% 
%     subplot(1,2,2)
%     plot3(model(1,:),model(2,:),model(3,:),'r.',dataOut(1,:),dataOut(2,:),dataOut(3,:),'g.'), axis equal
%     xlabel('x')
%     ylabel('y')
%     zlabel('z') 
    
    prePoint = totalPoint;
    j=j+1;
end
sum = 0;


dlmwrite('.\output\cor_matrix.txt',ref_matrix,'delimiter',',','newline','pc','precision','%.2f');
dlmwrite('.\output\T_matrix.txt',T_matrix,'delimiter',',','newline','pc','precision','%.2f');
dlmwrite('.\output\R_matrix.txt',R_matrix,'delimiter',',','newline','pc','precision','%.2f');
dlmwrite('.\output\frame.txt',new_frame,'delimiter',',','newline','pc','precision','%.0f');
dlmwrite('.\output\frameC.txt',frameC,'delimiter',',','newline','pc','precision','%.0f');
fprintf("finish");

%%
%------------------------------------------------------------------------------------
clc;
clear;

% Rotation and translation matrix
load('.\output\cor_matrix.txt');
load('.\output\T_matrix.txt');
load('.\output\R_matrix.txt');
% point cloud data indormation
load('.\output\cor_matrix.txt');
load('.\output\frame.txt');
load('.\output\frameC.txt');

new_matrix = cor_matrix;

isOpti = 1;



E_length=0;
V_set =5;


% Create Point
    % set up the initial location, the origin [0,0,0]
    data = [0;
        0;
        0];
  
    
pre_data = data;
position = zeros(3,length(T_matrix)+1);

% Set up the measured value for the inner wall, outer wall, and path.
x = 0:0.01:8.8;
ref_size = length(x);
ref = zeros(2,ref_size*3);

x = 1:0.01:7.8;
ref_size = length(x);
inw = zeros(2,ref_size*3);


x = -1:0.01:9.8;
ref_size = length(x);
ouw = zeros(2,ref_size*3);


flage =  1;
flage1 =  1;
flage2 =  1;
for i = 1:4
    for j = 0:0.01:8.8
        if i == 1
           ref(2,flage) = j;
           flage = flage + 1;
        elseif i == 2 
            ref(1,flage) = j;
            ref(2,flage) = 8.8;
            flage = flage + 1;
        elseif i ==3
            ref(1,flage) = 8.8;
            ref(2,flage) = 8.8 - j;
            flage = flage + 1;
        elseif i ==4
            ref(1,flage) = 8.8 - j;
            ref(2,flage) = 0;
            flage = flage + 1;
        end

    end
    for j = 1:0.01:7.8
        if i == 1
           inw(1,flage1) = 1;
           inw(2,flage1) = j;
           flage1 = flage1 + 1;
        elseif i == 2 
            inw(1,flage1) = j;
            inw(2,flage1) = 7.8;
            flage1 = flage1 + 1;
        elseif i ==3
            inw(1,flage1) = 7.8;
            inw(2,flage1) = 7.8 - j + 1;
            flage1 = flage1 + 1;
        elseif i ==4
            inw(1,flage1) = 7.8 - j + 1;
            inw(2,flage1) = 1;
            flage1 = flage1 + 1;
        end
    end
    for j = -1:0.01:9.8
        if i == 1
           ouw(1,flage2) = -1;
           ouw(2,flage2) = j;
           flage2 = flage2 + 1;
        elseif i == 2 
            ouw(1,flage2) = j;
            ouw(2,flage2) = 9.8;
            flage2 = flage2 + 1;
        elseif i ==3
            ouw(1,flage2) = 9.8;
            ouw(2,flage2) = 9.8 - j - 1;
            flage2 = flage2 + 1;
        elseif i ==4
            ouw(1,flage2) = 9.8 - j - 1;
            ouw(2,flage2) = -1;
            flage2 = flage2 + 1;
        end
    end
    
end

inw_size = length(inw);
ouw_size = length(ouw);
wall = zeros(2,inw_size+ouw_size+1);
wall(:,1:inw_size) = inw;
wall(:,(inw_size+1):(inw_size+ouw_size)) = ouw;

%--------------------------------------------------

% figure(1);
% plot3(ref(1,:),ref(2,:),ref(3,:),'-','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
% 
% title('Full trajectory')
% xlabel('x')
% ylabel('y')
% zlabel('z')



atFrame = 1;
RTSUM = [1,0,0;
         0,1,0;
         0,0,1];
TSUM = [0;
        0;
        0];
     
     

zSum = zeros(1,length(frameC));

% Loop over the matrix to work out the displacement information
% and work out the trajectory  in 3D space.
for i = 1:length(T_matrix)

    %--------------------------------------------------
    if i == 1
        Ri = 1;
    else
        Ri = (i-1) *3;
        Ri = Ri +1;
    end
    RT = zeros(3,3);
    RT(1,:) = R_matrix(1,Ri:Ri+2) ;
    RT(2,:) = R_matrix(2,Ri:Ri+2) ;
    RT(3,:) = R_matrix(3,Ri:Ri+2) ;
   
    TT = zeros(3,1);
    TT(1,:) = T_matrix(1,i);
    TT(2,:) = T_matrix(2,i);
    TT(3,:) = T_matrix(3,i);
    %----------------------------------------------
    
    % cumulate the translation and roation.
    RTSUM =  RT * RTSUM ;
    TSUM =  TSUM + (RTSUM*TT);


    % D + SRT
    TT = RTSUM * TT;
    data = data + TT;
         
    position(:,i+1) = data;
end
    
  
             
    location = zeros(3,length(position));
    
    RTSUM = [1,0,0;
            0,1,0;
            0,0,1];
        
    TSUM = [0;
            0;
             0];
    sw = 1;
    
    % loop over each position and do the optimization
for  j = 1:length(position)
    
    % Calculate the average distance between each point 
    % Will be used to calibrate the trajectory.
    ave = calculateAveEu(position);
    
    for i = 1:length(position)-1
        if( (i == j) && (j > 1) )
            % Check if the difference between the current and next point on
            % the trajectory larger than the threshold.
            % If the difference larger than the threshold, then the
            % previous value will be assigned.
            
            %  The code here may looks messy as it was designed 
            %  not only to check the difference between current and next position 
            %  but also to check the current and previous position. 
            if( (calculateDiff(i,position,ave) ~= 0) && (sw == 1)&&(isOpti == 1))
                 sw = 0;
                if i == 1
                    Ri = 1;
                else
                    Ri = (i-1) *3;
                    Ri = Ri +1;
                end
                
                if( calculateDiff(i,position,ave) == 1 ) 
                    Rp = Ri -3;
                    Ti = i - 1;
                else
                    Rp = Ri +3;
                    Ti = i + 1;
                end
                R_matrix(1,Ri:Ri+2) = R_matrix(1,Rp:Rp+2);
                R_matrix(2,Ri:Ri+2) = R_matrix(2,Rp:Rp+2);
                R_matrix(3,Ri:Ri+2) = R_matrix(3,Rp:Rp+2);


                T_matrix(1,i) = T_matrix(1,Ti);
                T_matrix(2,i) = T_matrix(2,Ti);
                T_matrix(3,i) = T_matrix(3,Ti);


            end
        end


        % Re-calcualte the position information based on the optimized
        % translation and rotation matrix.
        if i == 1
            Ri = 1;
        else
            Ri = (i-1) *3;
            Ri = Ri +1;
        end

        RT = zeros(3,3);
        RT(1,:) = R_matrix(1,Ri:Ri+2) ;
        RT(2,:) = R_matrix(2,Ri:Ri+2) ;
        RT(3,:) = R_matrix(3,Ri:Ri+2) ;

        TT = zeros(3,1);
        TT(1,:) = T_matrix(1,i);
        TT(2,:) = T_matrix(2,i);
        TT(3,:) = T_matrix(3,i);

        RTSUM =  RT * RTSUM ;
        TSUM =  TSUM + (RTSUM*TT);

        % 5: D + SRT
        TT = RTSUM * TT;
        location(:,i+1) = location(:,i) + TT;

        %----------------------------------------------
    end
    position = location;    
    figure(2)
    plot3(position(1,:),position(2,:),position(3,:),'ro-'),axis equal,grid on;
     xlabel('x')
     ylabel('y')
     zlabel('z')
    RTSUM = [1,0,0;
            0,1,0;
            0,0,1];
        
    TSUM = [0;
            0;
             0];
    sw = 1;
    flage = 1;

end 

% %----------------------test
% data = cor_matrix(:,1:200);
% ref_data = data;
% RTSUM = [1,0,0;
%          0,1,0;
%          0,0,1];
% TSUM = [0;
%         0;
%          0];
%      
% for i = 1:length(T_matrix)
% 
%     %--------------------------------------------------
%     if i == 1
%         Ri = 1;
%     else
%         Ri = (i-1) *3;
%         Ri = Ri +1;
%     end
%     RT = zeros(3,3);
%     RT(1,:) = R_matrix(1,Ri:Ri+2) ;
%     RT(2,:) = R_matrix(2,Ri:Ri+2) ;
%     RT(3,:) = R_matrix(3,Ri:Ri+2) ;
%    
%     TT = zeros(3,1);
%     TT(1,:) = T_matrix(1,i);
%     TT(2,:) = T_matrix(2,i);
%     TT(3,:) = T_matrix(3,i);
%     %----------------------------------------------
%     
%     RTSUM =  RT * RTSUM ;
%     TSUM =  TSUM + (RTSUM*TT);
% 
% 
%     % 5: D + SRT
%     data = RTSUM * data;
%          
%     
%     figure(9)
%     title('Original data points (blue) and rotated points (red)')
%     
%     plot3(data(1,:),data(2,:),data(3,:),'r.'), axis equal, hold on;
%    
%     plot3(ref_data(1,:),ref_data(2,:),ref_data(3,:),'b.'), axis equal, hold off;
%     xlabel('x')
%      ylabel('y')
%      zlabel('z')
% end
% 
% %----------------------test

D2_matrix = position;

% 2D projection
[D2_matrix] = matrixProjection(D2_matrix,1);
figure(3)
plot(D2_matrix(1,:),D2_matrix(2,:),'bo-'),axis equal,grid on, hold on;
plot(ref(1,:),ref(2,:),'-','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
plot(inw(1,:),inw(2,:),'-','Color','k','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
plot(ouw(1,:),ouw(2,:),'-','Color','k','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on

% work out the 2D displacement information
displacement = calculateDisplacement(D2_matrix);


% %--------------sorting the angle
% 
% for i = 1: length(displacement)
%     
% end
% 
% %------------------------------

D2_pos = zeros(2,length(D2_matrix));
D2_pos(1,:) = D2_matrix(1,:);
D2_pos(2,:) = D2_matrix(2,:);

passedPoint = 1;


theta = -90;
point = [];
land = [];
ref_matrix = [];
measurement = [];
% Work out the 2D measurement, the 2D projection of the trajectory has been
% done in pervious section. This section is projection the 3D environment
% measurement in to 2D format. 
for i = 1:length(frameC)
    
    cur_matrix = zeros(3,frameC(i));
    cur_matrix(1,:) = new_matrix(1,passedPoint:frameC(i)+passedPoint-1);
    cur_matrix(2,:) = new_matrix(2,passedPoint:frameC(i)+passedPoint-1);
    cur_matrix(3,:) = new_matrix(3,passedPoint:frameC(i)+passedPoint-1);
    %---------Filtering---------
    %      zero_frame: the frames that be filtered
    [zero_frame] = pointFilter (cur_matrix(1,:),cur_matrix(2,:),cur_matrix(3,:),0,0,2);
    %--------------------------Build up matrix
    % producing the matrix contain the x y and z value corresponding to the sorted frame 
    cur_matrix = buildUpMatrix(cur_matrix(1,:),cur_matrix(2,:),cur_matrix(3,:),zero_frame);
    %--------------------------------------
%     ref_matrix = addMatrix(ref_matrix,cur_matrix);
    
    mt = calculateD2Measurement(cur_matrix);
    measurement = addMatrix(measurement,mt);
    
    theta = theta + displacement(3,i);
    
   RT = [cos(theta),sin(theta);
         -sin(theta), cos(theta)];
     
     cur_matrix(1:2,:) = RT *  cur_matrix(1:2,:) ;

     
     cur_matrix(1,:) = cur_matrix(1,:)+ D2_pos(1,i);
     cur_matrix(2,:) = cur_matrix(2,:)+ D2_pos(2,i);
     point = addMatrix(point,cur_matrix(1:2,:));
    
    label = zeros(1,length(cur_matrix));
    label(:) = i;
    land = addMatrix(land,label);
    
    figure(5);
    plot(ref(1,:),ref(2,:),'-','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
    plot(inw(1,:),inw(2,:),'-','Color','k','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
    plot(ouw(1,:),ouw(2,:),'-','Color','k','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
    plot(cur_matrix(1,:),cur_matrix(2,:),'b.'),axis equal, hold on;
    plot(D2_matrix(1,:),D2_matrix(2,:),'bo-'),axis equal,hold on;
    passedPoint = passedPoint+frameC(i)-1;
end
frameC = zeros(1,i);
for j = 1:length(land)
    frameC(land(j)) = frameC(land(j)) + 1;
end
    for j = 1:length(displacement)
        displacement(3,j) = displacement(3,j) * (pi/180);
        displacement(4,j) = displacement(4,j) * (pi/180);
    end
    for j = 1:length(measurement)
        measurement(2,j) = measurement(2,j) * (pi/180);
    end
    theta = 15;
    theta = theta * (pi/180);
    rot = [cos(theta),sin(theta);
         -sin(theta), cos(theta)];
     figure(9)
     D2_pos = rot * D2_pos;
     point = rot * point;
     plot(ref(1,:),ref(2,:),'-','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
     plot(inw(1,:),inw(2,:),'-','Color','k','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
     plot(ouw(1,:),ouw(2,:),'-','Color','k','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
     plot(D2_pos(1,:),D2_pos(2,:),'ro-'),axis equal,hold on;
     plot(point(1,:),point(2,:),'r.'),axis equal,hold on;
     figure(8)
     plot(D2_pos(1,:),D2_pos(2,:),'ro-'),axis equal,hold on;
     plot(ref(1,:),ref(2,:),'-','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
     plot(inw(1,:),inw(2,:),'-','Color','k','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
     plot(ouw(1,:),ouw(2,:),'-','Color','k','MarkerSize',5,'MarkerFaceColor','#D9FFFF'),axis equal,hold on
    [aRMS,aDistance] = evaluate(ref,D2_pos);
    [aWallRMS,~] = evaluate (wall,point);
   
    
%     startPoint = 1;
%     for i = 1:length(frameC)
%         figure(10);
%         plot(point(1,startPoint:startPoint+frameC(i)),point(2,startPoint:startPoint+frameC(i)),'r.'),axis equal,grid on,hold on;
%         startPoint = startPoint + frameC(i);
%     end
    
    
dlmwrite('.\output\frameC_new.txt',frameC,'delimiter',',','newline','pc','precision','%.0f');
dlmwrite('.\output\dx.txt',displacement(1,:),'delimiter',',','newline','pc','precision','%.3f');
dlmwrite('.\output\dy.txt',displacement(2,:),'delimiter',',','newline','pc','precision','%.3f');
dlmwrite('.\output\loc.txt',displacement(3,:),'delimiter',',','newline','pc','precision','%.3f');
dlmwrite('.\output\glob.txt',displacement(4,:),'delimiter',',','newline','pc','precision','%.3f');
dlmwrite('.\output\poses_x.txt',D2_pos(1,:),'delimiter',',','newline','pc','precision','%.3f');
dlmwrite('.\output\poses_y.txt',D2_pos(2,:),'delimiter',',','newline','pc','precision','%.3f');
dlmwrite('.\output\points_x.txt',point(1,:),'delimiter',',','newline','pc','precision','%.3f');
dlmwrite('.\output\points_y.txt',point(2,:),'delimiter',',','newline','pc','precision','%.3f');
% dlmwrite('landmark_x.txt',ref_matrix(1,:),'delimiter',',','newline','pc','precision','%.3f');
% dlmwrite('landmark_y.txt',ref_matrix(2,:),'delimiter',',','newline','pc','precision','%.3f');
dlmwrite('.\output\measurement_r.txt',measurement(1,:),'delimiter',',','newline','pc','precision','%.3f');
dlmwrite('.\output\measurement_a.txt',measurement(2,:),'delimiter',',','newline','pc','precision','%.3f');

fprintf("finish2");