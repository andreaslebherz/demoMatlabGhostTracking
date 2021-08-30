function [posError,velError,dimError,yawError] = helperTargetError(track,truth)
% Errors as a function of target track and associated truth.

% Get true information from the ground truth.
truePos = truth.Position(1:2)';
% Position is at the rear axle for all vehicles. We would like to compute
% the error from the center of the vehicle
rot = [cosd(truth.Yaw) -sind(truth.Yaw);sind(truth.Yaw) cosd(truth.Yaw)];
truePos = truePos + rot*[truth.Wheelbase/2;0];

trueVel = truth.Velocity(1:2);
trueYaw = truth.Yaw(:);
trueDims = [truth.Length;truth.Width];

% Get estimated value from track. 
% GGIW-PHD tracker outputs a struct field 'Extent' and 'SourceIndex' 
% GM-PHD tracker outputs struct with but not 'Extent'
% multiObjectTracker outputs objectTrack

if isa(track,'objectTrack')
    estPos = track.State([1 3]);
    estVel = track.State([2 4]);
    % No yaw or dimension information in multiObjectTracker.
    estYaw = nan;
    estDims = [nan;nan];
elseif isfield(track,'Extent') % trackerPHD with GGIWPHD
    estPos = track.State([1 3]);
    estVel = track.State([2 4]);
    estYaw = atan2d(estVel(2),estVel(1));
    d = eig(track.Extent);
    dims = 2*sqrt(d);
    estDims = [max(dims);min(dims)];
else % trackerPHD with GMPHD
    estPos = track.State(1:2);
    estYaw = track.State(4);
    estVel = [track.State(3)*cosd(estYaw);track.State(3)*sind(estYaw)];
    estDims = track.State(6:7);
end

% Compute 2-norm of error for each attribute.
posError = norm(truePos(:) - estPos(:));
velError = norm(trueVel(:) - estVel(:));
dimError = norm(trueDims(:) - estDims(:));
yawError = norm(trueYaw(:) - estYaw(:));
end