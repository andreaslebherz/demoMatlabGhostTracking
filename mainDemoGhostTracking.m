clc;
clear;
close all;

%% Define constants
EVAL_POT = true;
EVAL_JPDA = true;
PLOT_METRICS = true;
EXPORT_CSV = true;
n_radar = 5;
id_ego = 1;

%% Load scenario and init plot window

% Create the scenario
addpath('scenarios');

% Handle for exported driving scenario
% scenarioHandle = @demo_ghost_detections;
scenarioHandle = @demo_ghost_detections_2;
% scenarioHandle = @demo_ghost_highway_ramp;
filename = 'demo_ghost_detections_2';

% Create scenario-obj, ego-vehicle motion and sensor setup
[scenario, egoVehicle, sensors] = helperCreateScenario(scenarioHandle);

numRadar = sum(cellfun(@(s) isa(s, 'radarDataGenerator'), sensors, 'UniformOutput', true));
% sanity check
assert(id_ego==egoVehicle.ActorID, sprintf('Ego vehicle needs ID: %d.', id_ego));
assert(n_radar==numRadar, sprintf('Setup needs a number of sensors: %d.', n_radar));

% Create the display object
display = helperTargetTrackingDisplay;
% Set this to vehicle ID you want to follow
display.FollowActorID = 2;

% Create the Animation writer to record each frame of the figure for
% animation writing. Set 'RecordGIF' to true to enable GIF writing.
gifWriter = helperGIFWriter('Figure',display.Figure,...
    'RecordGIF',true);

% Create output folder if it does not exist
if ~exist('output', 'dir')
   mkdir('output')
end

%% Define error metrics
% Function to return the errors given track and truth.
errorFcn = @(track,truth)helperTargetError(track,truth);

% Function to return the distance between track and truth
distFcn = @(track,truth)helperTargetDistance(track,truth);

% Function to return the IDs from the ground truth. The default
% identifier assumes that the truth is identified with PlatformID. In
% drivingScenario, truth is identified with an ActorID.
truthIdFcn = @(x)[x.ActorID];

% Create metrics object.
tem = trackErrorMetrics(...
    'ErrorFunctionFormat','custom',...
    'EstimationErrorLabels',{'PositionError','VelocityError','DimensionsError','YawError'},...
    'EstimationErrorFcn',errorFcn,...
    'TruthIdentifierFcn',truthIdFcn);

tam = trackAssignmentMetrics(...
    'DistanceFunctionFormat','custom',...
    'AssignmentDistanceFcn',distFcn,...
    'DivergenceDistanceFcn',distFcn,...
    'TruthIdentifierFcn',truthIdFcn,...
    'AssignmentThreshold',30,...
    'DivergenceThreshold',35);

% Create ospa metric object
tom = trackOSPAMetric(...
    'Distance','custom',...
    'DistanceFcn',distFcn,...
    'TruthIdentifierFcn',truthIdFcn);

%% Point Object Tracker GNN
% Reset the random number generator for repeatable results
seed = 2021;
S = rng(seed);

trackerRunTimes = zeros(0,2);
ospaMetric = zeros(0,2);

if EVAL_POT
    timeStep = 1;
    % Create a multiObjectTracker
    tracker = multiObjectTracker(...
        'FilterInitializationFcn', @helperInitPointFilter, ...
        'AssignmentThreshold', 30, ...
        'ConfirmationThreshold', [4 5], ...
        'DeletionThreshold', 3);

    % Tranformation is necessary for correct plotting
    for i = 1:numRadar
            sensors{i}.HasRangeRate = false;
            sensors{i}.DetectionCoordinates = 'Body';
            sensors{i}.TargetReportFormat = 'Detections';
            sensors{i}.HasElevation = false;
    end

    % Run the scenario
    while advance(scenario) && ishghandle(display.Figure)
        % Get the scenario time
        time = scenario.SimulationTime;

        % Collect detections from the ego vehicle sensors
        [detections,isValidTime] = helperDetect(sensors, egoVehicle, time);

        % Update the tracker if there are new detections
        if any(isValidTime)
            % Detections must be clustered first for the point tracker
            detectionClusters = helperClusterRadarDetections(detections, numRadar);
            % Update the tracker
            tic
            % confirmedTracks are in scenario coordinates
            confirmedTracks = updateTracks(tracker, detectionClusters, time);
            t = toc;

            % Update the metrics
            % a. Obtain ground truth
            groundTruth = scenario.Actors(2:end); % All except Ego

            % b. Update assignment metrics
            tam(confirmedTracks,groundTruth);
            [trackIDs,truthIDs] = currentAssignment(tam);

            % c. Update error metrics
            tem(confirmedTracks,trackIDs,groundTruth,truthIDs);

            % d. Update ospa metric
            ospaMetric(timeStep,1) = tom(confirmedTracks, groundTruth);

            % Update bird's-eye-plot
            % Convert tracks to ego coordinates for display
            confirmedTracksEgo = helperConvertToEgoCoordinates(egoVehicle, confirmedTracks);
            
            % Display detections
            display(egoVehicle, sensors, detections, confirmedTracksEgo, detectionClusters);
            drawnow;
            
            if EXPORT_CSV
                export_sensor_data(scenario, detections, egoVehicle, strcat('output/GNN_', filename));
            end
            % Record tracker run times
            trackerRunTimes(timeStep,1) = t;
            timeStep = timeStep + 1;

            % Capture frames for animation
            gifWriter();
        end
    end
    
    % Capture the cumulative track metrics. The error metrics show the averaged
    % value of the error over the simulation.
    assignmentMetricsMOT = tam.trackMetricsTable;
    errorMetricsMOT = tem.cumulativeTruthMetrics;

    % Write GIF
    writeAnimation(gifWriter, strcat('output/GNN_', filename));
end

%% Point Object Tracker JPDA
% Reset the random number generator for repeatable results
seed = 2021;
S = rng(seed);

if EVAL_JPDA
    % Create a multiObjectTracker
    tracker = trackerJPDA(...
        'FilterInitializationFcn', @helperInitPointFilter, ...
        'AssignmentThreshold', 30, ...
        'ConfirmationThreshold', [4 5], ...
        'DeletionThreshold', 3);
    
    % Release and restart all objects.
    restart(scenario);
    release(tem);
    release(tam);
    release(display);
    gifWriter.pFrames = {};
    
    % For multiObjectTracker, the radar reports in Ego Cartesian frame and does
    % not report velocity. This allows us to cluster detections from multiple
    % sensors.
    for i = 1:numRadar
        release(sensors{i});
        sensors{i}.HasRangeRate = false;
        sensors{i}.DetectionCoordinates = 'Body';
        sensors{i}.TargetReportFormat = 'Detections';
        sensors{i}.HasElevation = false;
    end
        
    % Restore random seed.
    rng(seed)
    % First time step
    timeStep = 1;
    %%%
    % Run the scenario
    while advance(scenario) && ishghandle(display.Figure)
        % Get the scenario time
        time = scenario.SimulationTime;

        % Collect detections from the ego vehicle sensors
        [detections,isValidTime] = helperDetect(sensors, egoVehicle, time);
        
        % Update the tracker if there are new detections
        if any(isValidTime)
            % Detections must be clustered first for the point tracker
            detectionClusters = helperClusterRadarDetections(detections, numRadar);

            % Update the tracker
            tic
            % confirmedTracks are in scenario coordinates
            confirmedTracks = tracker(detectionClusters, time);
            t = toc;

            % Update the metrics
            % a. Obtain ground truth
            groundTruth = scenario.Actors(2:end); % All except Ego

            % b. Update assignment metrics
            tam(confirmedTracks,groundTruth);
            [trackIDs,truthIDs] = currentAssignment(tam);

            % c. Update error metrics
            tem(confirmedTracks,trackIDs,groundTruth,truthIDs);

            % d. Update ospa metric
            ospaMetric(timeStep,2) = tom(confirmedTracks, groundTruth);

            % Update bird's-eye-plot
            % Convert tracks to ego coordinates for display
            confirmedTracksEgo = helperConvertToEgoCoordinates(egoVehicle, confirmedTracks);
            display(egoVehicle, sensors, detections, confirmedTracksEgo, detectionClusters);
            drawnow;
            
            if EXPORT_CSV
                export_sensor_data(scenario, detections, egoVehicle, strcat('output/JPDA_', filename));
            end

            % Record tracker run times
            trackerRunTimes(timeStep,2) = t;
            timeStep = timeStep + 1;

            % Capture frames for animation
            gifWriter();
        end
    end
    
    % Capture the cumulative track metrics. The error metrics show the averaged
    % value of the error over the simulation. 
    assignmentMetricsJPDA = tam.trackMetricsTable;
    errorMetricsJPDA = tem.cumulativeTruthMetrics;

    % Write GIF
    writeAnimation(gifWriter, strcat('output/JPDA_', filename));
end

%%
% *Plot metrics*
if PLOT_METRICS
    helperPlotAssignmentMetrics(assignmentMetricsMOT, assignmentMetricsJPDA);
    helperPlotErrorMetrics(errorMetricsMOT, errorMetricsJPDA);
    
    ospaFig = figure;
    plot(ospaMetric,'LineWidth',2);
    legend('Point Target Tracker GNN', 'Point Target Tracker JPDA');
    xlabel('Time step (k)');
    ylabel('OSPA');
    
    runTimeFig = figure;
    h = plot(trackerRunTimes(3:end,:)./trackerRunTimes(3:end,1),'LineWidth',2);
    legend('Point Target Tracker GNN', 'Point Target Tracker JPDA');
    xlabel('Time step (k)');
    ylabel('$$\frac{t_{tracker}}{t_{multiObjectTracker}}$$','interpreter','latex','fontsize',14);
    ylim([0 max([h.YData]) + 1]);
end