% % mmWave 3D Point Clouds SLAM
clear;
clc;
close all;

% Load and format data 
filename = "../../Sensor data/PC data/Data (short room)/ShortRoomStraightLine_1.csv";
% Read data and variables
opts = detectImportOptions(filename);
opts.SelectedVariableNames = {'x', 'y', 'z'};
data = readtable(filename, opts);
data = table2array(data);

% Erase weird characters from string before processing
data = erase(data, newline);
data = erase(data, char(13)); % Carriage return

% Sorting data by columns into different vectors
xData = data(:, 1);
yData = data(:, 2);
zData = data(:, 3);

xOutData = [];
yOutData = [];
zOutData = [];
pClouds = {};

% Set the number of frames to merge
frames = 20;

% Loop through data and combine set number of frames
for i=1:height(data)
    xOutData = [xOutData str2num(xData{i})];
    yOutData = [yOutData str2num(yData{i})];
    zOutData = [zOutData str2num(zData{i})];
    if (mod(i, frames) == 0)
        pClouds{int32(i)/frames} = [xOutData', yOutData', zOutData'];
        xOutData = [];
        yOutData = [];
        zOutData = [];
    end
    
end

% pClouds = n-by-3 array

% Parameters For Point Cloud Registration Algorithm
% We should update these parameters by experimenting with mmWave Sensor

maxRange = 8.19; % Modified parameter

referenceVector = [0 0 1];
maxDistance = 0.1; % Modified parameter
maxAngularDistance = 15;

randomSampleRatio = 1; % Modified parameter

% The following parameters have been modified for each motion

% Straight line
gridStep = 2;
distanceMovedThreshold = 0.5;

% L shape
% gridStep = 5;
% distanceMovedThreshold = 0.01;

% U shape
% gridStep = 5;
% distanceMovedThreshold = 0.01;

% Parameters For Loop Closure Estimation Algorithm

loopClosureSearchRadius = 0.01;

nScansPerSubmap = 1;
subMapThresh = 10;

annularRegionLimits = [-0.75 0.75];

rmseThreshold = 5;

loopClosureThreshold = 50;
optimizationInterval = 2;

% Initialize Variables

% 3D Posegraph object for storing estimated relative poses
pGraph = poseGraph3D;
% Default serialized upper-right triangle of 6-by-6 Information Matrix
infoMat = [1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1];
% Number of loop closure edges added since last pose graph optimization and map refinement
numLoopClosuresSinceLastOptimization = 0; 
% True after pose graph optimization until the next scan
mapUpdated = false;
% Equals to 1 if the scan is accepted
scanAccepted = 0;

% 3D Occupancy grid object for creating and visualizing 3D map
mapResolution = 50; % cells per meter
omap = occupancyMap3D(mapResolution);

pcProcessed = cell(1,length(pClouds));
mmWaveScans2d = cell(1,length(pClouds)); 
submaps = cell(1,int32(length(pClouds)/nScansPerSubmap));

pcsToView = cell(1,length(pClouds)); 

% Set to 1 to visualize created map and posegraph during build process
viewMap = 1; 
% Set to 1 to visualize processed point clouds during build process
viewPC = 1;

rng(0);

% If you want to view the point clouds while processing them sequentially
if viewPC==1
    pplayer = pcplayer([-10 10],[-10 10],[-10 10],'MarkerSize',10);
end

% If you want to view the created map and posegraph during build process
if viewMap==1
    ax = newplot; % Figure axis handle
    view(10,15);
    grid on;
end


% Trajectory Estimation And Refinement Using Pose Graph Optimization

count = 0; % Counter to track number of scans added
disp('Estimating robot trajectory...');

for i=1:length(pClouds)
    % Read point clouds in sequence
    disp('Read point clouds in sequence');
    pc = pClouds{i};

    ind = (-maxRange < pc(:,1) & pc(:,1) < maxRange ...
        & -maxRange  < pc(:,2) & pc(:,2) < maxRange ...
        & (abs(pc(:,2))>abs(0.5*pc(:,1)) | pc(:,1)>0));
    
    pcl = pointCloud(pc(ind,:));

    [~, ~, outliers] = ...
        pcfitplane(pcl, maxDistance,referenceVector,maxAngularDistance);
    pcl_wogrd = select(pcl,outliers,'OutputSize','full');

    ind = (pcl_wogrd.Location(:,3)<annularRegionLimits(2))&(pcl_wogrd.Location(:,3)>annularRegionLimits(1));
    pcl_wogrd = select(pcl_wogrd,ind,'OutputSize','full');

    pcl_wogrd_sampled = pcdownsample(pcl_wogrd,'random',randomSampleRatio);
    
    if viewPC==1
        % Visualize down sampled point cloud
        disp('Visualize down sampled point cloud');
        view(pplayer,pcl_wogrd_sampled);
        pause(0.5)
    end    

    
    if count == 0
        % First scan
        disp('First scan');
        tform = [];
        scanAccepted = 1;
    else
        if count == 1
            disp('Count = 1');
            tform = pcregisterndt(pcl_wogrd_sampled,prevPc,gridStep);
        else
            disp('Count > 1');
            tform = pcregisterndt(pcl_wogrd_sampled,prevPc,gridStep,...
                'InitialTransform',prevTform);
        end
        
        relPose = [tform2trvec(tform.T') tform2quat(tform.T')];
        
        if sqrt(norm(relPose(1:3))) > distanceMovedThreshold
            addRelativePose(pGraph,relPose);
            scanAccepted = 1;
            disp('Scan accepted');
        else
            scanAccepted = 0;
            disp('Scan NOT accepted');
        end
    end
 
    if scanAccepted == 1
        disp('Scan accepted, increment count');
        count = count + 1;
        
        pcProcessed{count} = pcl_wogrd_sampled;
        
        mmWaveScans2d{count} = helperCreate2DScan(pcl_wogrd_sampled);
        
        % Submaps are created for faster loop closure query. 
        if rem(count,nScansPerSubmap)==0
            disp('Creating submaps');
            submaps{count/nScansPerSubmap} = helperCreateSubmap(mmWaveScans2d,...
                pGraph,count,nScansPerSubmap,maxRange);
        end
        
        % loopSubmapIds contains matching submap ids if any otherwise empty.   
        if (floor(count/nScansPerSubmap)>subMapThresh)
            disp('Matching submaps ID');
            [loopSubmapIds,~] = helperEstimateLoopCandidates(pGraph,...
                count,submaps,mmWaveScans2d{count},nScansPerSubmap,...
                loopClosureSearchRadius,loopClosureThreshold,subMapThresh);
            
            if ~isempty(loopSubmapIds)
                rmseMin = inf;
                disp('Estimate best match to the current scan');
                % Estimate best match to the current scan
                for k = 1:length(loopSubmapIds)
                    % For every scan within the submap
                    for j = 1:nScansPerSubmap
                        probableLoopCandidate = ...
                            loopSubmapIds(k)*nScansPerSubmap - j + 1;
                        [loopTform,~,rmse] = pcregisterndt(pcl_wogrd_sampled,...
                            pcProcessed{probableLoopCandidate},gridStep);
                        % Update best Loop Closure Candidate
                        if rmse < rmseMin
                            loopCandidate = probableLoopCandidate;
                            rmseMin = rmse;
                        end
                        if rmseMin < rmseThreshold
                            break;
                        end
                    end
                end
                
                % Check if loop candidate is valid
                if rmseMin < rmseThreshold
                    disp('Check if loop candidate is valid');
                    % loop closure constraint
                    relPose = [tform2trvec(loopTform.T') tform2quat(loopTform.T')];
                    
                    addRelativePose(pGraph,relPose,infoMat,...
                        loopCandidate,count);
                    numLoopClosuresSinceLastOptimization = numLoopClosuresSinceLastOptimization + 1;
                end
                     
            end
        end

        if (numLoopClosuresSinceLastOptimization == optimizationInterval)||...
                ((numLoopClosuresSinceLastOptimization>0)&&(i==length(pClouds)))
            disp('Optimization Interval');
            if loopClosureSearchRadius ~=1
                disp('Doing Pose Graph Optimization to reduce drift.');
            end
            % pose graph optimization
            pGraph = optimizePoseGraph(pGraph);
            loopClosureSearchRadius = 1;
            if viewMap == 1
                position = pGraph.nodes;
                % Rebuild map after pose graph optimization
                omap = occupancyMap3D(mapResolution);
                for n = 1:(pGraph.NumNodes-1)
                    insertPointCloud(omap,position(n,:),pcsToView{n}.removeInvalidPoints,maxRange);
                end
                mapUpdated = true;
                ax = newplot;
                grid on;
            end
            numLoopClosuresSinceLastOptimization = 0;
            % Reduce the frequency of optimization after optimizing
            % the trjectory
            optimizationInterval = optimizationInterval*7;
        end 

        pcToView = pcdownsample(pcl_wogrd_sampled, 'random', 1);
        pcsToView{count} = pcToView;
        
        if viewMap==1
            % Insert point cloud to the occupance map in the right position
            disp('Insert point cloud to the occupance map in the right position');
            position = pGraph.nodes(count);
            insertPointCloud(omap,position,pcToView.removeInvalidPoints,maxRange);
            
            if (rem(count-1,1)==0)||mapUpdated
                helperVisualizeMapAndPoseGraph(omap, pGraph, ax);
                disp('Map updated with helper function helperVisualizeMapAndPoseGraph');
            end
            mapUpdated = false;
        else
            % Give feedback to know that example is running
            disp('Give feedback to know that example is running');
            if (rem(count-1,1)==0)
                fprintf('.');
            end
        end
        
        prevPc = pcl_wogrd_sampled;
        prevTform = tform;
    end
end

set(gca, 'Color', 'w')
