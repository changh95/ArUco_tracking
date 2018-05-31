%%Information

% This code lets you detect ArUco markers using 1 camera.
% input = camera(0)
% output = vecs (column 1: total time, column 2:n, 4x4 transformation matrix for marker number in ascending order.

%% Before using this code

% 1. calibrate the camera and generate camera_parameters.mat
% 2. specify marker length (use tape measure on printed markers)
% 3. check dictionary IDs. (start from ID(0) to ID(N))
%% Parameters
clear

% options
markerLength = 0.009        % Marker side length (in meters). Needed for correct scale in camera pose
dictionaryId = '6x6_250';  % Dictionary id
showRejected = false;      % Show rejected candidates too
estimatePose = true;      % Wheather to estimate pose or not
load camera_parameters.mat -mat camMatrix distCoeffs

% marker detector parameters
detectorParams = struct();
%if false
    %detectorParams.nMarkers = 1024;
    detectorParams.adaptiveThreshWinSizeMin = 3;
    detectorParams.adaptiveThreshWinSizeMax = 23;
    detectorParams.adaptiveThreshWinSizeStep = 10;
    detectorParams.adaptiveThreshConstant = 7;
    detectorParams.minMarkerPerimeterRate = 0.03;
    detectorParams.maxMarkerPerimeterRate = 4.0;
    detectorParams.polygonalApproxAccuracyRate = 0.05;
    detectorParams.minCornerDistanceRate = 0.05;
    detectorParams.minDistanceToBorder = 3;
    detectorParams.minMarkerDistanceRate = 0.05;
    detectorParams.cornerRefinementMethod = 'Subpix';
    detectorParams.cornerRefinementWinSize = 5;
    detectorParams.cornerRefinementMaxIterations = 30;
    detectorParams.cornerRefinementMinAccuracy = 0.1;
    detectorParams.markerBorderBits = 1;
    detectorParams.perspectiveRemovePixelPerCell = 8;
    detectorParams.perspectiveRemoveIgnoredMarginPerCell = 0.13;
    detectorParams.maxErroneousBitsInBorderRate = 0.04;
    detectorParams.minOtsuStdDev = 5.0;
    detectorParams.errorCorrectionRate = 1.0;
%end
detectorParams.cornerRefinementMethod = 'Subpix';  % do corner refinement in markers
dictionary = {'Predefined', dictionaryId};
%% Input source
    vid = cv.VideoCapture(0);
    waitTime = 0.00001;  % 10 msec
%% Main loop
totalTime = 0;
totalIterations = 0;
hImg = [];
t=tic();
vec =[];
times = [];
vecs = [];
while true
    % read frame
    
    img = vid.read();
    if  isempty(img)
        break;
    end
    
    tId = tic();
    
    % detect markers and estimate pose
    [corners, ids] = cv.detectMarkers(img, dictionary, 'DetectorParameters',detectorParams);
    
    if estimatePose && ~isempty(ids)
        [rvecs, tvecs] = cv.estimatePoseSingleMarkers(corners,markerLength, camMatrix, distCoeffs);

        for index = 1:1:size(ids,2)
            temp = {rotm2tform(cv.Rodrigues(rvecs{1,index}))+trvec2tform(tvecs{1,index}) - [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 1]};
            vec = [vec temp];
        end
        vec = array2table(vec');
        ids_1 = array2table(ids');
        vec.Properties.VariableNames = {'matrix'};
        ids_1.Properties.VariableNames = {'ids'};
        ids_vec = [ids_1 vec];
        ids_vec = sortrows(ids_vec,1);
        if ids_vec{1,1} ~=0
            zero_NaN_row = [0 NaN];
            zero_NaN_row = array2table(zero_NaN_row);
            zero_NaN_row.Properties.VariableNames = {'ids','matrix'};
            ids_vec = [zero_NaN_row;ids_vec];
        end
        for index = 1:1:size(ids_vec,1)-1
            if ids_vec{index+1,1}-ids_vec{index,1} ~= 1
                temp = [index NaN];
                temp = array2table(temp);
                temp.Properties.VariableNames = {'ids','matrix'};
                ids_vec = [ids_vec; temp];
                ids_vec = sortrows(ids_vec,1);
            end
        end
        vec = ids_vec.matrix;
        vec = vec';
        vecs = [vecs;totalTime vec];
        vec=[];
    end
    
    % tic/toc
    currentTime = toc(tId);
    totalTime = totalTime + currentTime
    totalIterations = totalIterations + 1;
    if mod(totalIterations, 30) == 0
        fprintf('Detection time = %f ms (Mean = %f ms)\n', ...
            1000*currentTime, 1000*totalTime/totalIterations);
    end
    
    % draw results
    if ~isempty(ids)
        img = cv.drawDetectedMarkers(img, corners, 'IDs',ids);
        if estimatePose
            for i=1:numel(ids)
                img = cv.drawAxis(img, camMatrix, distCoeffs, ...
                    rvecs{i}, tvecs{i}, markerLength * 0.5);
            end
        end
    end
    
    %  if showRejected && ~isempty(rejected)
    %     img = cv.drawDetectedMarkers(img, rejected, 'BorderColor',[255 0 100]);
    %  end
    
    
    if isempty(hImg)
        hImg = imshow(img);
        
    elseif ishghandle(hImg)
        set(hImg, 'CData',img);
        
    else
        break;
    end
    drawnow;
end
vid.release();
%% time fix
totalTime_real=toc(t)
time_Ratio = totalTime_real/totalTime;
for index=1:size(vecs)
    vecs(index,1)={(time_Ratio*cell2mat(vecs(index,1)))};
end


