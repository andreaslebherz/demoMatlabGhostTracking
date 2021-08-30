classdef helperTargetTrackingDisplay < matlab.System
    % This is a helper class for Object Tracking example to
    % display tracks, detections and ground truth. It may be removed or
    % modified in a future release.
    
    % Copyright 2019 The MathWorks, Inc.
    
    % Public properties
    properties
        Figure
        BirdsEyePlots
        % ActorID to follow in the center panel
        FollowActorID = [];
        % Plot clustered radar detections?
        PlotClusteredDetection = true;
    end
    
    methods
        function obj = helperTargetTrackingDisplay(varargin)
            setProperties(obj,nargin,varargin{:});
            % Make a figure
            hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Demonstration Ghost-Detections');
            set(hFigure,'Visible','off')
            movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top
            obj.Figure = hFigure;
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, egoVehicle, sensors)
            obj.BirdsEyePlots = createDemoDisplay(obj,egoVehicle,sensors);
        end
        
        function stepImpl(obj, egoVehicle, ~,  detections, tracks, varargin)
            bep = obj.BirdsEyePlots;
            % Update plots
            helperUpdateDisplay(bep, egoVehicle, detections, tracks, varargin{:});
            
            % Follow actor ID provided in actor ID
            if ~isempty(obj.FollowActorID)
                scene = egoVehicle.Scenario;
                pos = targetOutlines(egoVehicle);
                actorPos = pos(ismember([scene.Actors.ActorID],obj.FollowActorID),:);
                minX = min(actorPos(:,1));
                maxX = max(actorPos(:,1));
                minY = min(actorPos(:,2));
                maxY = max(actorPos(:,2));
                bep{2}.XLimits = [minX-25 maxX+25];
                bep{2}.YLimits = [minY-15 maxY+15];
            end
        end
        
        function BEPS = createDemoDisplay(obj, egoCar, sensors)
            hFigure = obj.Figure;
            
            % Add a car plot that follows the ego vehicle from behind
            hCarViewPanel = uipanel(hFigure, 'Position', [0.005 0.75 0.25 0.25], 'Title', 'Chase Camera View');
            hCarPlot = axes(hCarViewPanel);
            chasePlot(egoCar, 'Parent', hCarPlot);
            
            % Create panels with bird's-eye plots
            BEP = createBEPPanel(obj, hFigure, [0.005 0 0.25 0.75], 60, sensors, 'Bird''s-Eye Plot', false);
            FullBEP = createBEPPanel(obj, hFigure, [0.63 0 0.37 1], 80, {}, 'Followed Vehicle', true);
            CenterBEP = createBEPPanel(obj, hFigure, [0.255 0 0.37 1], 40, {}, 'Ego Vehicle', true);
            BEPS = {BEP,FullBEP,CenterBEP};
            if isempty(snapnow('get'))
                set(hFigure,'Visible','on')
            end
        end
    end
end


function BEP = createBEPPanel(obj, hFigure, position, frontBackLim, sensors, title, isLegend)
    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', position, 'Title', title);
    
    % Create bird's-eye plot for the ego car and sensor coverage
    hBEVPlot = axes(hBEVPanel);
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);
    
    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-frontBackLim frontBackLim]*0.667);
    legend(hBEVPlot, 'off')
    
    % Create a vision detection plotter
    detectionPlotter(BEP, 'DisplayName','vision','MarkerEdgeColor','blue', 'MarkerFaceColor','blue','Marker','^','MarkerSize',6);
    
    % Create a radar detection plotter - standard
    detectionPlotter(BEP, 'DisplayName','radar', 'Marker','o','MarkerEdgeColor','black','MarkerFaceColor','red','MarkerSize',4);
    
    % Create a radar detection plotter - first bounce
    detectionPlotter(BEP, 'DisplayName','radar-b1', 'Marker','o','MarkerEdgeColor','black','MarkerFaceColor','yellow','MarkerSize',4);
    
    % Create a radar detection plotter - second bounce
    detectionPlotter(BEP, 'DisplayName','radar-b2', 'Marker','o','MarkerEdgeColor','black','MarkerFaceColor','magenta','MarkerSize',4);
    
    if obj.PlotClusteredDetection
        % Create a clustered detection plotter
        trackPlotter(BEP, 'DisplayName','radar cluster', 'Marker','*','MarkerEdgeColor','red','MarkerFaceColor','red','MarkerSize',4);
    end
    
    % Create a lane marking plotter
    laneMarkingPlotter(BEP, 'DisplayName','lane');
    
    % Create a track plotter
    trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);
    
    % Add an outline plotter for ground truth
    outlinePlotter(BEP, 'Tag', 'Ground truth');
    
    numRadar = sum(cellfun(@(s) isa(s, 'radarDataGenerator'), sensors, 'UniformOutput', true));
    
    if ~isempty(sensors)
        % Plot the coverage areas for radars
        for i = 1:numRadar
            cap = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
            plotCoverageArea(cap, sensors{i}.MountingLocation(1:2),...
                sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(1), sensors{i}.FieldOfView(1));
        end

        % Plot the coverage areas for vision sensors
        for i = numRadar+1:numel(sensors)
            cap = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
            plotCoverageArea(cap, sensors{i}.SensorLocation,...
                sensors{i}.MaxRange, sensors{i}.Yaw, 45);
        end
    end
    
    if ~isLegend
        
    else
        legend('Orientation','horizontal','NumColumns',3)
        legend('Location', 'NorthOutside')
    end
end

function helperUpdateDisplay(BEPS, egoCar, detections, confirmedTracks, varargin)
%%% 
% helperUpdateDisplay  Helper to update the display with tracks
% 
% This function updates the bird's-eye plot with road boundaries,
% detections, and tracks.
    for b = 1:numel(BEPS)
        BEP = BEPS{b};
        helperUpdateDisplayNonTracks(BEP, egoCar, detections, varargin{:});
        trackIDs = [confirmedTracks.TrackID];
        if isa(confirmedTracks,'objectTrack') % multiObjectTracker
            [trackPos, trackPosCov] = getTrackPositions(confirmedTracks,[1 0 0 0 0;0 0 1 0 0]);
            plotTrack(findPlotter(BEP,'DisplayName','track'), trackPos, trackPosCov, string(trackIDs));
        end
    end
end


function [position, yaw, length, width] = tracksOutlines(tracks)
% tracksOutlines  Returns the track outlines for display
    position = zeros(numel(tracks), 2);
    yaw = zeros(numel(tracks), 1);
    length = zeros(numel(tracks), 1);
    width = zeros(numel(tracks), 1);

    for i = 1:numel(tracks)
        position(i, :) = tracks(i).State(1:2)';
        yaw(i) = tracks(i).State(4);
        length(i, 1) = tracks(i).State(6);
        width(i, 1) = tracks(i).State(7);
    end
end

function helperUpdateDisplayNonTracks(BEP, egoCar, detections, detectionCluster)
%helperUpdateDisplayNonTracks  Helper to update display of all non-track plotters

    % Update road boundaries and their display
    [lmv, lmf] = laneMarkingVertices(egoCar);
    plotLaneMarking(findPlotter(BEP,'DisplayName','lane'),lmv,lmf);

    phi = egoCar.Yaw;
    rot = [cosd(phi), sind(phi); -sind(phi), cosd(phi)];
    % update ground truth data
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);

    % Prepare and update detections display
    sIdx = cellfun(@(x)x.SensorIndex,detections);
    uqSIdx = unique(sIdx);
    posEgo = zeros(3,numel(detections));
    isRadar = sIdx <= 5;

    is_bounce0 = [cellfun(@(x) x.ObjectAttributes{1}.BouncePathIndex == 0, detections(isRadar)),; zeros(numel(sIdx(~isRadar), 1))] ;
    is_bounce1 = [cellfun(@(x) x.ObjectAttributes{1}.BouncePathIndex == 1, detections(isRadar)),; zeros(numel(sIdx(~isRadar), 1))] ;
    is_bounce2 = [cellfun(@(x) x.ObjectAttributes{1}.BouncePathIndex == 2, detections(isRadar)),; zeros(numel(sIdx(~isRadar), 1))] ;

    % convert from double to logical
    is_bounce0 = is_bounce0 == 1;
    is_bounce1 = is_bounce1 == 1;
    is_bounce2 = is_bounce2 == 1;

    for i = 1:numel(uqSIdx)
        thisIdx = sIdx == uqSIdx(i);
        posEgo(:,thisIdx) = calculatePositionInEgoFrame(detections(thisIdx));
    end

    plotDetection(findPlotter(BEP,'DisplayName','vision'), posEgo(1:2,~isRadar)');
    plotDetection(findPlotter(BEP,'DisplayName','radar'), posEgo(1:2,is_bounce0)');
    plotDetection(findPlotter(BEP,'DisplayName','radar-b1'), posEgo(1:2,is_bounce1)');
    plotDetection(findPlotter(BEP,'DisplayName','radar-b2'), posEgo(1:2,is_bounce2)');

    if nargin > 3
        sIdx = cellfun(@(x)x.SensorIndex,detectionCluster);
        uqSIdx = unique(sIdx);
        posEgo = zeros(3,numel(detectionCluster));
        posCovEgo = zeros(3,3,numel(detectionCluster));
        isRadar = sIdx <= 5;
        for i = 1:numel(uqSIdx)
            thisIdx = sIdx == uqSIdx(i);
            [posEgo(:,thisIdx),~,posCovEgo(:,:,thisIdx)] = calculatePositionInEgoFrame(detectionCluster(thisIdx));
        end
        plotTrack(findPlotter(BEP,'DisplayName','radar cluster'), posEgo(1:2,isRadar)', posCovEgo(1:2,1:2,isRadar));
    end
end

function [posEgo, velEgo, posCovEgo] = calculatePositionInEgoFrame(detections)

% Calculate Cartesian positions for all detections in the "sensor"
% coordinate frame
allDets = [detections{:}];
meas = horzcat(allDets.Measurement);

if strcmpi(allDets(1).MeasurementParameters(1).Frame,'Spherical')
    az = meas(1,:);
    r = meas(2,:);
    el = zeros(1,numel(az));
    [x, y, z] = sph2cart(deg2rad(az),deg2rad(el),r);
    posSensor = [x;y;z];
    rr = meas(3,:);
    rVec = posSensor./sqrt(dot(posSensor,posSensor,1));
    velSensor = rr.*rVec;
else
    posSensor = meas;
    velSensor = zeros(3,size(meas,2));
end

% Transform parameters
sensorToEgo = detections{1}.MeasurementParameters(1);
R = sensorToEgo.Orientation;
T = sensorToEgo.OriginPosition;
if isfield(sensorToEgo,'OriginVelocity')
    Tdot = sensorToEgo.OriginVelocity;
else
    Tdot = zeros(3,1);
end

if isfield(sensorToEgo,'IsParentToChild') && sensorToEgo.IsParentToChild
    R = R';
end

% Position, velocity in ego frame
posEgo = T + R*posSensor;
velEgo = Tdot + R*velSensor; % Assume Rdot = 0;

if nargout > 2
    assert(~strcmpi(allDets(1).MeasurementParameters(1).Frame,'Spherical'),'Only cartesian measurements');
    measCov = cat(3,allDets.MeasurementNoise);
    posCovEgo = zeros(3,3,numel(allDets));
    for i = 1:numel(allDets)
        posCovEgo(:,:,i) = R*measCov(:,:,i)*R';
    end
end
end

