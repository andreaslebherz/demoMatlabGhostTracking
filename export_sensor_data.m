function export_sensor_data(scenario, detections, egoVehicle, filename)
    % create .csv files with headers
    [cfilename, gtfilename] = create_csv_files(filename);
    
    % Get current time
    current_time = posixtime(datetime(2021, 01, 01, 00, 00, 00, 00));
    
    % sensor data
    % 'timestamp', 'sid', 'x_sd', 'y_sd', 'z_sd', 'vx_sd', 'vy_sd', 'vz_sd', 
    ctime = cellfun(@(x) x.Time, detections);            
    ctime = ctime + repmat(current_time, size(detections,1), 1);
    csid = cellfun(@(x) x.SensorIndex, detections);

    % all measurements
    cmeas = cellfun(@(x) x.Measurement, detections, 'UniformOutput', false);
    % create mask for camera sensor
    mask = cellfun(@(x) size(x.Measurement,1) == 3, detections);
    % append zeros as velocity information of camera sensor
    cmeas(mask) = cellfun(@(x) [x; zeros(3,1)], cmeas(mask), 'UniformOutput', false);
    % create array from comma-seperated list
    cmeas = [cmeas{:}]';
    % add position of ego vehicle to measured position offset
    cmeas = cmeas + repmat([egoVehicle.Position, 0,0,0], size(cmeas,1), 1);

    % create mask to seperate radar and camera sensors
    radar_mask = cellfun(@(x) x.SensorIndex < 6, detections);
    % split detections
    radar_detections = detections(radar_mask);
    camera_detections = detections(~radar_mask);

    % extract SNR property from radar sensors
    radar_snr = cellfun(@(x) x.ObjectAttributes{1}.SNR, radar_detections);
    SNR = [radar_snr; ones(size(camera_detections)) * (-1)];            

    % ground truth data
    % 'timestamp', 'oid', 'x_gt', 'y_gt', 'z_gt' 'vx_gt', 'vy_gt',
    % 'vz_gt', SNR
    num_cartesian_components = 3;
    num_actors = size(scenario.Actors, 2);
    cpgt = zeros(num_actors, num_cartesian_components);
    cvgt = zeros(num_actors, num_cartesian_components);
    gtoid = zeros(num_actors, 1);
    gttime = strings(num_actors, 1);
    for i = 1:size(scenario.Actors, 2)
        gttime(i,:) = scenario.SimulationTime + current_time;
        gtoid(i,:) = scenario.Actors(1,i).ActorID;
        cpgt(i,:) = scenario.Actors(1,i).Position;
        cvgt(i,:) = scenario.Actors(1,i).Velocity;  
    end

    cout = cat(2, ctime, csid, cmeas, SNR);
    gtout = cat(2, gttime, gtoid, cpgt, cvgt);
    %write data to end of files
    writematrix(cout, cfilename, 'Delimiter', ',', 'WriteMode','append');
    writematrix(gtout, gtfilename, 'Delimiter', ',', 'WriteMode','append');
end


function [cfilename, gtfilename] = create_csv_files(filename)
% generate .csv file for sensor data
    cfilename = strcat(filename, '.csv');
    cHeader = {'timestamp', 'sid', ...
        'x_sd', 'y_sd', 'z_sd', ...
        'vx_sd', 'vy_sd', 'vz_sd', 'SNR'};
    ctextHeader = strjoin(cHeader, ',');
    %write header to file
    cfid = fopen(cfilename,'w');
    fprintf(cfid,'%s\n', ctextHeader);
    fclose(cfid);
    
    % generate .csv file for ground truth
    gtfilename = strcat(filename, '_ground_truth.csv');
    gtHeader = {'timestamp', 'oid', ...  
        'x_gt', 'y_gt', 'z_gt',...
        'vx_gt', 'vy_gt', 'vz_gt'};    
    gttextHeader = strjoin(gtHeader, ',');
    gtfid = fopen(gtfilename,'w');    
    %write header to file
    fprintf(gtfid,'%s\n', gttextHeader);
    fclose(gtfid);
end