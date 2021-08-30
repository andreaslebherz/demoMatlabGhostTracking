function [detections, isValidTime, transientConfigs] = helperDetect(sensors, egoVehicle, time, sensorConfigs)
%helperDetect  Helper function to collect detections from all sensors

%   Copyright 2018 The MathWorks, Inc.

% Simulate the sensors and collect detections
detections = {};
isValidTime = false(1,numel(sensors));

ta = targetPoses(egoVehicle);

for i = 1:numel(sensors)
    % Detections are reported in sensor spherical frame for radar and
    % sensor cartesian frame for camera.
    [sensorDets, numValidDets, config] = sensors{i}(ta, time);
    
    % radar returns struct, vision returns boolean
    if isa(config, 'struct')
        isValidTime(i) = config.IsValidTime;
    else 
        isValidTime(i) = config;
    end
    
    % isValidTime is true when reporting any detections.
    isValidTime(i) = isValidTime(i) && ~isempty(sensorDets);
    
    % The radar measurement noise is equal to the resolution of the sensor.
    if isa(sensors{i},'radarDataGenerator')
        hasRangeRate = sensors{i}.HasRangeRate;
        for k = 1:numValidDets
            if strcmpi(sensorDets{k}.MeasurementParameters(1).Frame,'spherical')
                if hasRangeRate
                    sensorDets{k}.MeasurementNoise = diag((1/2*[sensors{i}.AzimuthResolution sensors{i}.RangeResolution sensors{i}.RangeRateResolution]).^2);
                else
                    sensorDets{k}.MeasurementNoise = diag(([sensors{i}.AzimuthResolution sensors{i}.RangeResolution]/2).^2);
                end
            else
                % x y noise set to range resolution for Cartesian frame.
                if hasRangeRate
                    sensorDets{k}.MeasurementNoise = diag([sensors{i}.RangeResolution sensors{i}.RangeResolution 0.4 10 10 10].^2);
                else
                    sensorDets{k}.MeasurementNoise = diag([sensors{i}.RangeResolution sensors{i}.RangeResolution 0.4].^2);
                end
            end
        end
    end
    
    % The camera measurement noise is roughly set to a reasonable value
    if isa(sensors{i},'visionDetectionGenerator')
        for k = 1:numValidDets
            % Only raw position measurement is used. filtered velocity
            % measurement is removed. 
            sensorDets{k}.Measurement = sensorDets{k}.Measurement(1:3);
            sensorDets{k}.MeasurementNoise = sensorDets{k}.MeasurementNoise(1:3,1:3);
            sensorDets{k}.MeasurementNoise(1:3,1:3) = blkdiag(0.5*eye(2),0.4);
            sensorDets{k}.MeasurementParameters = sensorDets{k}.MeasurementParameters{1};
            sensorDets{k}.MeasurementParameters(1).HasVelocity = false;
        end
    end
    detections = [detections;sensorDets];  %#ok<AGROW>
end

egoPos = egoVehicle.Position(:);
egoVel = egoVehicle.Velocity(:);
egoOrient = rotmat(quaternion([egoVehicle.Yaw egoVehicle.Pitch egoVehicle.Roll],'eulerd','ZYX','frame'),'frame');
if numel(detections) > 0
    % Add information about egoVehicle
    for i = 1:numel(detections)
        % The sensors do not move with respect to ego vehicle
        detections{i}.MeasurementParameters(1).OriginVelocity = zeros(3,1); %#ok<AGROW>
        
        % Parameters for scenario to ego transformation
        egoParams = detections{i}.MeasurementParameters(1);
        egoParams.Frame = drivingCoordinateFrameType.Rectangular;
        egoParams.OriginPosition = egoPos;
        egoParams.OriginVelocity = egoVel;
        egoParams.Orientation = egoOrient';
        detections{i}.MeasurementParameters = [detections{i}.MeasurementParameters(1);egoParams]; %#ok<AGROW>
    end
end

if nargout > 2
    % Part of the configuration that changes with time to update with
    % trackerPHD.
    transientConfigs = struct('SensorTransformParameters',{},'IsValidTime',{},'SensorIndex',{});

    for i = 1:numel(sensorConfigs)
        transientConfigs(i).SensorTransformParameters = sensorConfigs{i}.SensorTransformParameters;
        transientConfigs(i).SensorTransformParameters(2).OriginPosition = egoPos;
        transientConfigs(i).SensorTransformParameters(2).OriginVelocity = egoVel;
        transientConfigs(i).SensorTransformParameters(2).Orientation = egoOrient;
        transientConfigs(i).IsValidTime = isValidTime(i);
        transientConfigs(i).SensorIndex = i;
    end
end

end