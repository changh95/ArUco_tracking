%% ArUco marker detection using 2 cameras
% Author: Hyunggi Chang (h.chang17@imperial.ac.uk)
% Equipment: 2x Microsoft LifeCam Studio
%
% input : vid, vid_2 (videos from camera 1 and 2), marker length, marker IDs
% output = cam_1, cam_2 (timestamp, rvec and tvec of each marker.)
%
% Anything written between ** that are commented, are the points that need
% to be improved.
%% How to use
% 1. Make sure camera 1 and 2 are intrinsically calibrated, and
% stereocalibrated.
% 2. Measure the marker side length
% 3. Record total number of marker IDs.
% 4. Update 3 and 4 on the code.
% 5. Run the code.
%% Parameters

% Options
% *Implement QuestionDialogue feature to input marker length and totalIds*

markerLength = 0.0125        % Marker side length (in meters). Needed for correct scale in camera pose
totalIds = 9 % Marker IDs: 0, 1, 2, 3, 4, 5 
dictionaryId = '6x6_250';  % Dictionary ID
showRejected = false;      % Show rejected candidates too (dev feature)
estimatePose = true;      % Wheather to estimate pose or not (dev feature)
load camera_parameters_1.mat -mat camMatrix_cam_1 distCoeffs_cam_1 % Load Intrinsic Camera Parameters cam_1
load camera_parameters_2.mat -mat camMatrix_cam_2 distCoeffs_cam_2 % Load Intrinsic Camera Parameters cam_2

% Marker detector parameters
detectorParams = struct();
    detectorParams.adaptiveThreshWinSizeMin = 3;
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
dictionary = {'Predefined', dictionaryId};

%% Input source

%Initiate videocapture
vid = cv.VideoCapture(0);
vid.set('FrameWidth',1280)
vid.set('FrameHeight',720)
vid_2 = cv.VideoCapture(2);
vid_2.set('FrameWidth',1280)
vid_2.set('FrameHeight',720)

%% Main loop
totalTime = 0;
hImg_1 = []; hImg_2 = [];
cam_1 = []; cam_2 = [];
t=tic();

while true
    % Read frames
    img_1 = vid.read();
    img_2 = vid_2.read();
    if  isempty(img_1) && isempty(img_2), break; end
  
    tId = tic(); % tic

    % Collect corner and ID information
    [corners_1, ids_1] = cv.detectMarkers(img_1, dictionary, 'DetectorParameters',detectorParams);
    [corners_2, ids_2] = cv.detectMarkers(img_2, dictionary, 'DetectorParameters',detectorParams);
    
    % Estimate Pose (cam_1)
    if estimatePose && ~isempty(ids_1)
        % Collect rvecs and tvecs
        [rvecs_1, tvecs_1] = cv.estimatePoseSingleMarkers(corners_1,markerLength, camMatrix_cam_1, distCoeffs_cam_1);
        
        % Check if any IDs are repeated (if IDs are repeated, there is a singularity error causing double reading)
        for i = 1:numel(ids_1)
            singularity_check(1,i) = sum(ids_1(:) == i-1) <= 1;
        end
        
        % Register rvecs/tvecs to marker IDs, and arrange in ascending order.
        if any(singularity_check)
            for i = 1:totalIds
                vec{1,i} = [rvecs_1(ids_1==i-1), tvecs_1(ids_1==i-1)]; % Variable 'vec' has rvecs and tvecs
                if isempty(vec{1,i})
                    vec{1,i}=NaN; % Any non-detected markers have NaN
                end
            end
            cam_1 = [cam_1 ; totalTime vec(1,:)];
        end
    end

    % Estimate Pose (cam_2)
    if estimatePose && ~isempty(ids_2)
        % Collect rvecs and tvecs
        [rvecs_2, tvecs_2] = cv.estimatePoseSingleMarkers(corners_2,markerLength, camMatrix_cam_2, distCoeffs_cam_2);
        
        % Check if any IDs are repeated (if IDs are repeated, there is a singularity error causing double reading)
        for i = 1:numel(ids_2)
            singularity_check(1,i) = sum(ids_2(:) == i-1) <= 1;
        end
        
        % Register rvecs/tvecs to marker IDs, and arrange in ascending order.
        if any(singularity_check)
            for i = 1:totalIds
                vec{1,i} = [rvecs_2(ids_2==i-1), tvecs_2(ids_2==i-1)]; % Variable 'vec' has rvecs and tvecs
                if isempty(vec{1,i})
                    vec{1,i}=NaN; % Any non-detected markers have NaN
                end
            end
            cam_2 = [cam_2 ; totalTime vec(1,:)];
        end
    end

    % Read stopwatch
    currentTime = toc(tId);
    totalTime = totalTime + currentTime

    % Draw results (Visualising tracking is enabled, in expense of huge
    % frame rate reduction. Disable the feature when programming is done)
    if ~isempty(ids_1)
       img_1 = cv.drawDetectedMarkers(img_1, corners_1, 'IDs',ids_1);
        if estimatePose
           for i=1:numel(ids_1)
                img_1 = cv.drawAxis(img_1, camMatrix_cam_1, distCoeffs_cam_1, ...
                    rvecs_1{i}, tvecs_1{i}, markerLength * 0.5);
            end            
        end
    end
    
    if ~isempty(ids_2)
         img_2 = cv.drawDetectedMarkers(img_2, corners_2, 'IDs',ids_2);
        if estimatePose
            for i=1:numel(ids_2)
                 img_2 = cv.drawAxis(img_2, camMatrix_cam_2, distCoeffs_cam_2, ...
                    rvecs_2{i}, tvecs_2{i}, markerLength * 0.5);
            end
        end
    end
 
    if isempty(hImg_1) && isempty(hImg_2) 
        figure(1);
        hImg_1 = imshow(img_1);
        figure(2);
        hImg_2 = imshow(img_2);
    elseif ishghandle(hImg_1) && ishghandle(hImg_2)
        set(hImg_1, 'CData',img_1);
       set(hImg_2, 'CData', img_2);
   else
        break;
    end
    drawnow;
end
% End videocapture and close the figures
vid.release();
vid_2.release();
close (hImg_1);
close (hImg_2);
%% Time fix
totalTime_real=toc(t)
time_Ratio = totalTime_real/totalTime;
for i=1:size(cam_1)
    cam_1(i,1)={(time_Ratio*cell2mat(cam_1(i,1)))};
end
for i=1:size(cam_2)
    cam_2(i,1)={(time_Ratio*cell2mat(cam_2(i,1)))};
end

%% Tracking data filter (Outlier rejection)
% *Currently excluded from the code.*