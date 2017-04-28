%% Constants
disparityRange = [0,32];
blockSize = 13;
ref = 2;

%% size of grid filter for preprocessing
gridSize = 5;

%% The overlapped region is filtered using a 1.5cm box grid filter. Increase the
    % merge size to reduce the storage requirement of the resulting scene point
    % cloud, and decrease the merge size to increase the scene resolution.
mergeSize = 0.015;

% load('stereoParams.mat')
%% Reference Image


%%Load Reference Image
refImageFileName1 = ['new/col_capture_',num2str(ref),'_1.png'];
refImageFileName2 = ['new/col_capture_',num2str(ref),'_2.png'];
refereceImage1 = imread(refImageFileName1);
referenceImage2 = imread(refImageFileName2);

%% Rectify Images
[rectifiedRefImage1, rectifiedRefImage2] = rectifyStereoImages(refereceImage1,referenceImage2,stereoParams);


%% Show the images overlapped 
% figure
% imshow(cat(3,rectifiedRefImage1(:,:,1),rectifiedRefImage2(:,:,2:3)),'InitialMagnification',50);

%% Compute Disparity
% In rectified stereo images any pair of corresponding points are located 
% on the same pixel row. For each pixel in the left image compute the
% distance to the corresponding pixel in the right image. This distance is
% called the disparity, and it is proportional to the distance
frameLeftGrayRef  = rgb2gray(rectifiedRefImage1);
frameRightGrayRef = rgb2gray(rectifiedRefImage2);
tic;
disparityMapRef = disparity(frameLeftGrayRef, frameRightGrayRef,'BlockSize',...
    blockSize, 'DisparityRange',disparityRange);
toc
% %% Show Disparity Map
%     figure
%     imshow(disparityMapRef,disparityRange);
%     title('Disparity Map');
%     colormap jet
%     colorbar
% 
% %% Save Disparity Map
%     refDisparsityfileName = ['new/Depth_', num2str(ref)];
%     saveas(gcf,refDisparsityfileName, 'png');

%% Reconstruct the 3-D Scene
% Reconstruct the 3-D world coordinates of points corresponding to each
% pixel from the disparity map.
points3DRef = reconstructScene(disparityMapRef,stereoParams);

%% Convert to meters from centimeter and create a pointCloud object
% points3DRef = points3DRef .* 10;

%% Generate 3D PointCloud
ptCloudRef = pointCloud(points3DRef, 'Color', rectifiedRefImage1);
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
meanDepth1 = mean(fixed.Location, 1);
minDepth1 = min(fixed.Location);
maxDepth1 = max(fixed.Location);



% 
%% Save 3D Poinr Cloud
    fileNameRef = ['new/a000', num2str(ref)];
    pcwrite(fixed, fileNameRef,'Encoding','ascii');

%% Create a streaming point cloud viewer
    player3D = pcplayer([-3000, 3000], [-3000, 3000], [0, 6000], 'VerticalAxis', 'y', ...
        'VerticalAxisDir', 'down');


%% Visualize the point cloud
    view(player3D, fixed);

%% Iterations for all the target
% for i = 4:4
%     disp(i);
%     
%     %% Load Image
%     targetImageFileName1 = ['new/col_capture_',num2str(i),'_1.png'];
%     targetImage1 = imread(targetImageFileName1);
%     targetImageFileName2 = ['new/col_capture_',num2str(i),'_2.png'];
%     targetImage2 = imread(targetImageFileName2);
%     
%     %% Rectify Image
%     [targetRectImage1, targetRectImage2] = rectifyStereoImages(targetImage1,targetImage2,stereoParams);
% 
%     %% Show Overlapped Images
%     figure
%     imshow(cat(3,targetRectImage1(:,:,1),targetRectImage2(:,:,2:3)),'InitialMagnification',50);
% 
%     %% Compute Disparity
%     % In rectified stereo images any pair of corresponding points are located 
%     % on the same pixel row. For each pixel in the left image compute the
%     % distance to the corresponding pixel in the right image. This distance is
%     % called the disparity, and it is proportional to the distance of the
%     frameLeftGrayTarget  = rgb2gray(targetRectImage1);
%     frameRightGrayTarget = rgb2gray(targetRectImage2);
% 
%     disparityMap2 = disparity(frameLeftGrayTarget, frameRightGrayTarget, 'BlockSize',...
%     blockSize, 'DisparityRange',disparityRange);
%     
% %     %% Show Disparity Map
% %     figure
% %     imshow(disparityMap2,disparityRange);
% %     title('Disparity Map');
% %     colormap jet
% %     colorbar
% %     
% %     %% Save Disparity Map
% %     disparsityfileName = ['Outside/Disparity/Depth_', num2str(i)];
% %     saveas(gcf,disparsityfileName, 'png');
% 
%     %% Reconstruct the 3-D Scene
%     % Reconstruct the 3-D world coordinates of points corresponding to each
%     % pixel from the disparity map.
%     points3DTarget = reconstructScene(disparityMap2,stereoParams);
% 
%     %% Convert to meters from centimeter and create a pointCloud object
%     points3DTarget = points3DTarget ./ 100;
%     
%     %%Generate Target PointCloud
%     ptCloudTarget = pointCloud(points3DTarget, 'Color', rectifiedRefImage1);
%     moving = pcdownsample(ptCloudTarget, 'gridAverage', gridSize);
%     meanDepth = mean(moving.Location, 1);
%     minDeptht = min(moving.Location);
%     maxDeptht = max(moving.Location);
%     
%     
% %     %% Save Target Point Cloud
% %     fileName = ['Outside/PointCloud/PointCloud_', num2str(i)];
% %     pcwrite(moving, fileName,'PLYFormat','binary');
% % % 
% %     %% Create a streaming point cloud viewer
% %     player3D2 = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
% %         'VerticalAxisDir', 'down');
% %      
% %      %% Visualize the point cloud
% %     view(player3D2, ptCloudTarget);
% 
%     
%     %% Register Two Point Clouds
%     % The quality of registration depends on data noise and initial settings of
%     % the ICP algorithm. You can apply preprocessing steps to filter the noise
%     % or set initial property values appropriate for your data. Here,
%     % preprocess the data by downsampling with a box grid filter and set the
%     % size of grid filter to be 10cm. The grid filter divides the point cloud
%     % space into cubes. Points within each cube are combined into a single
%     % output point by averaging their X,Y,Z coordinates.
% 
%     
% 
%     % Note that the downsampling step does not only speed up the registration,
%     % but can also improve the accuracy.
%     %% Downsample for merging
%     
%     %% Downsample Point Clouds
%     
%    
% 
%     %% 
%     % To align the two point clouds, we use the ICP algorithm to estimate the
%     % 3-D rigid transformation on the downsampled data. We use the first point
%     % cloud as the reference and then apply the estimated transformation to the
%     % original second point cloud. We need to merge the scene point cloud with
%     % the aligned point cloud to process the overlapped points.
% 
%     %%
%     % Begin by finding the rigid transformation for aligning the second point
%     % cloud with the first point cloud. Use it to transform the second point
%     % cloud to the reference coordinate system defined by the first point
%     % cloud.
% 
%     [tform, movingReg, rmse, iteration, maxx, minn, maxkeepInlier, minKeepInlier, maxInlierDist, minInlierDist] = mypcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
%     ptCloudAligned = pctransform(ptCloudTarget,tform);
%     
%     %%
%     % We can now create the world scene with the registered data. The
%     % overlapped region is filtered using a 1.5cm box grid filter. Increase the
%     % merge size to reduce the storage requirement of the resulting scene point
%     % cloud, and decrease the merge size to increase the scene resolution.
% 
%     [ptCloudScene, range]  = mypcmerge(ptCloudRef, ptCloudAligned, mergeSize);
%     
% %     ptCloudScene.Location = ptCloudScene.Location*100;
% %     %% Visualize the Point Cloud.
% %     player3D4 = pcplayer([-6, 6], [-6, 6], [2, 16], 'VerticalAxis', 'y', ...
% %         'VerticalAxisDir', 'down');
% % 
% %     view(player3D4, ptCloudScene);
% %     
%     %% Get average Distance Between two Point Clouds
%     disp('Getting mean');
% %     totalDist = 0;
% %     minDist = inf;
% %     maxDist = -10;
% %     [fixed,indices1]= removeInvalidPoints(fixed);
% %     [moving,indices2]= removeInvalidPoints(moving);
% %     for j = 1 : fixed.Count
% %         point = fixed.Location(j,:);
% %         [indices,dist] = findNearestNeighbors(moving,point,1);
% %         totalDist = totalDist + dist;
% %         if dist < minDist
% %             minDist = dist;
% %         end
% %         if dist > maxDist
% %             maxDist = dist;
% %         end
% %     end
% %     meanDist = totalDist / fixed.Count;
% 
%     %% Log Data
% %     mat = [ref, i, meanDepth1, meanDepth, rmse, iteration, minKeepInlier, maxInlierDist, mergeSize,  ptCloudScene.Count, ptCloudScene.XLimits, ptCloudScene.YLimits, ptCloudScene.ZLimits, range];
% % %     mat = [ref, i, meanDepth];
% %    dlmwrite ('Outside/Parameter1.csv', mat, '-append');
%     
% %     ptCloudScene=pcdownsample(ptCloudScene, 'gridAverage', gridSize);
%     %% Save Final Point Cloud
%     fileName = ['new/NewPointCloud_',num2str(ref),'_', num2str(i)];
%     pcwrite(ptCloudScene, fileName,'Encoding','ascii');
% end