function detectionClusters = helperClusterRadarDetections(detections, numRadar)
% helperClusterRadarDetections  Helper to cluster detections in the example
% 
% This function merges multiple detections suspected to be of the same
% vehicle to a single detection. The function looks for detections that are
% within a Mahalanobis distance of 2.5 between each other. 

% Copyright 2019 The Mathworks, Inc.

% Get sensor IDs of detections
sIDs = cellfun(@(x)x.SensorIndex, detections);
detsToCluster = detections(sIDs <= numRadar);
detsVision = detections(sIDs > numRadar);

% All detections are assumed to be in the same coordinate system.
% We will use partitionDetections functions to cluster all detections
% within a given distance.

% partitionDetections uses Mahalanobis distance and we would like to use
% the Euclidian distance between detections. Therefore the measurement
% noise is set to identity before passing it to partitionDetections.
for i = 1:numel(detsToCluster)
    detsToCluster{i}.MeasurementNoise = eye(3);
end

% Cluster two detections if they are within 2.5 meters.
clusters = partitionDetections(detsToCluster,2.5); 

numClusters = max(clusters);
detectionClusters = cell(numClusters,1);

for i = 1:numClusters
    thisClusterDets = detsToCluster(clusters == i);
    thisClusterObjDets = [thisClusterDets{:}];
    allMeas = horzcat(thisClusterObjDets.Measurement);
    allMeasNoise = cat(3,thisClusterObjDets.MeasurementNoise);
    detectionClusters{i} = thisClusterDets{1};
    meanMeas = mean(allMeas,2);
    detectionClusters{i}.Measurement = meanMeas;
    e = allMeas - meanMeas;
    detectionClusters{i}.MeasurementNoise = mean(allMeasNoise,3) + e*e'/numel(thisClusterDets);
    
    detectionClusters{i}.ObjectAttributes{1} = rmfield(detectionClusters{i}.ObjectAttributes{1}, 'BounceTargetIndex');
    detectionClusters{i}.ObjectAttributes{1} = rmfield(detectionClusters{i}.ObjectAttributes{1}, 'BouncePathIndex');
end

% All fields if object attributes must match for each detection.
% visionDetectionGenerator does not report SNR. Add a new field.
for i = 1:numel(detsVision)
    detsVision{i}.ObjectAttributes{1}.SNR = nan;
end
detectionClusters = [detectionClusters;detsVision];