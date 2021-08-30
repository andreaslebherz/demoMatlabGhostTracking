function filter = helperInitPointFilter(detection)
% helperInitPointFilter  A function to initialize the point-target filter
% based on the constant turn-rate model for the Extended Object Tracking
% example

% Copyright 2019 The MathWorks, Inc.

% Use a 2-D constant turn-rate model to initialize a trackingEKF.

% Initialize the 3-D filter using |initctekf|. This function transforms the
% measurement to state-space and initialize the state using the convention
% [x;vx;y;vy;w;z;vz].
filter3D = initctekf(detection);

% Use the first 5 states to initialize the 2-D filter
filter = trackingEKF('State',filter3D.State(1:5),...
    'StateCovariance',filter3D.StateCovariance(1:5,1:5),...
    'HasAdditiveProcessNoise',false,...
    'ProcessNoise',diag([1 1 3]),...
    'StateTransitionFcn',@constturn,...
    'MeasurementFcn',@ctmeas,...
    'MeasurementJacobianFcn',@ctmeasjac,...
    'StateTransitionJacobianFcn',@constturnjac);

end