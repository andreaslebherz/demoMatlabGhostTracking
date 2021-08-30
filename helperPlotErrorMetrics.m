function helperPlotErrorMetrics(motMetrics, jpdaMetrics)
    % This is a helper function and may be modified or removed in a future release.
    
    % Copyright 2019 The MathWorks, Inc
    
    % New figure
    hFigure = figure('Position',[0 0 1152 480]);
    movegui(hFigure, 'center');
    
    motMetrics = motMetrics(ismember(motMetrics.TruthID, intersect(motMetrics.TruthID,jpdaMetrics.TruthID)),:);
    jpdaMetrics = jpdaMetrics(ismember(jpdaMetrics.TruthID, intersect(motMetrics.TruthID,jpdaMetrics.TruthID)),:);
    
    catgs = categorical(motMetrics.TruthID);
    positionErrors = [motMetrics.PositionError jpdaMetrics.PositionError];
    velocityErrors = [motMetrics.VelocityError jpdaMetrics.VelocityError];
    
    % Position error plot
    subplot(1,2,1);
    bar(catgs,positionErrors);
    title('Position Error')
    ylabel('Error (m)');
    xlabel('Truth ID');
    l = legend('Point Target Tracker GNN', 'Point Target Tracker JPDA');
    l.Orientation = 'horizontal';
    l.Position(1) = 0.35;
    l.Position(2) = 0.95;
          
    % Velocity error plot
    subplot(1,2,2);
    bar(catgs,velocityErrors);
    title('Velocity Error')
    ylabel('Error (m/s)');
    xlabel('Truth ID');
end