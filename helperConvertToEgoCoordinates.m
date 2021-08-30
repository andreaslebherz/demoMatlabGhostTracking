function tracks = helperConvertToEgoCoordinates(egoCar, tracks)
% helperConvertToEgoCoordinates
%
% This function converts the tracks from global coordinates to ego
% coordinates.

% Copyright 2018 The Mathworks, Inc.

x0 = egoCar.Position(1:2)';
v0 = egoCar.Velocity(1:2)';
phi = egoCar.Yaw;
rot = [cosd(phi), sind(phi); -sind(phi), cosd(phi)];
if isfield(tracks,'SourceIndex') && isfield(tracks,'Extent') % GGIW-PHD
    for i = 1:numel(tracks)
        trackPos = tracks(i).State([1 3]);
        trackVel = tracks(i).State([2 4]);
        yaw = atan2d(trackVel(2),trackVel(1));
        tracks(i).State([1 3]) = rot*(trackPos - x0);
        tracks(i).State([2 4]) = rot*(trackVel - v0);
        
        % The extent of the track is rotated assuming a constant turn-rate.
        % However, the "integrated" turn-rate does not correctly represent
        % the yaw angle. Here yaw angle is calculated using the velocity
        % and the extent is aligned with the direction of speed.
        [~,extent] = eig(tracks(i).Extent);
        dims = sort(diag(extent));
        extent = [dims(2) 0;0 dims(1)];
        rotE = [cosd(phi - yaw) sind(phi - yaw);-sind(phi - yaw) cosd(phi - yaw)];
        tracks(i).Extent = rotE*extent*rotE';
    end
elseif isfield(tracks,'SourceIndex') % GM-PHD
    for i = 1:numel(tracks)
        trackPos = tracks(i).State(1:2);
        yaw = tracks(i).State(4);
        trackVel = [tracks(i).State(3)*cosd(yaw);tracks(i).State(4)*sind(yaw)];
        tracks(i).State(1:2) = rot * (trackPos - x0);
        tracks(i).State(3) = norm(trackVel - v0);
        tracks(i).State(4) = tracks(i).State(4) - phi;
    end
else % multiObjectTracker with objectTrack output
    for i = 1:numel(tracks)
        trackPos = tracks(i).State([1 3]);
        trackVel = tracks(i).State([2 4]);
        tracks(i).State([1 3]) = rot*(trackPos - x0);
        tracks(i).State([2 4]) = rot*(trackVel - v0);
    end
end