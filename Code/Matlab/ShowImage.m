gridSize = 0.05;
filename = uigetfile('*.ply');
ptCloud = pcread(filename);
ptCloud1 = pcdownsample(ptCloud, 'gridAverage', gridSize);
% player3D4 = pcplayer([-3, 3], [-2, 2], [3, 8], 'VerticalAxis', 'y', ...
%         'VerticalAxisDir', 'down');
% 
%     % Visualize the point cloud
% view(player3D4, ptCloud);

% pc = pointCloud(filename)
% pc.plot

% figure
% pcshow(ptCloud)