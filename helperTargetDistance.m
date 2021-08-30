function dist = helperTargetDistance(track,truth)
% This function computes the distance between track and a truth.

% Copyright 2019-2020 The MathWorks, Inc.

% Errors in each aspect
[posError,velError,dimError,yawError] =  helperTargetError(track,truth);

% For multiObjectTracker, add a constant penalty for not estimating yaw
% and dimensions
if isnan(dimError)
    dimError = 1;
end
if isnan(yawError)
    yawError = 1;
end

% Distance is the sum of errors
dist = posError + velError + dimError + yawError;


end