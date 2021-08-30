function helperPlotAssignmentMetrics(motMetrics, jpdaMetrics)
    % This is a helper function and may be modified or removed in a future
    % release
    
    % Copyright 2019 The MathWorks, Inc.
    
    % New figure
    figure('Units','normalized','Position',[0.1 0.1 0.5 0.5]);
    
    % Extract data
    motTgt = ~isnan(motMetrics.AssignedTruthID);
    jpdaTgt = ~isnan(jpdaMetrics.AssignedTruthID);
    tgtTracks = [sum(motTgt) sum(jpdaTgt)];
    tgtTrackIDs = {string(motMetrics.TrackID(motTgt)) string(jpdaMetrics.TrackID(jpdaTgt))};
    
    motRedundant = motMetrics.RedundancyStatus | (~motMetrics.Surviving & motMetrics.RedundancyCount > 0);
    jpdaRedundant = jpdaMetrics.RedundancyStatus | (~jpdaMetrics.Surviving & jpdaMetrics.RedundancyCount > 0);
    numRedundantTracks = [sum(motRedundant) sum(jpdaRedundant)];
    redTrackIDs = {string(motMetrics.TrackID(motRedundant)) string(jpdaMetrics.TrackID(jpdaRedundant))};
    
    motFalse = ~(motTgt | motRedundant);
    jpdaFalse = ~(jpdaTgt | jpdaRedundant);
    
    numFalseTracks = [sum(motFalse) sum(jpdaFalse)];
    falseTrackIDs = {string(motMetrics.TrackID(motFalse)) string(jpdaMetrics.TrackID(jpdaFalse))};
        
    data = [tgtTracks; numFalseTracks; numRedundantTracks];
    h = bar(data);
    motTracks = [tgtTrackIDs(1);falseTrackIDs(1);redTrackIDs(1)];
    jpdaTracks = [tgtTrackIDs(2);falseTrackIDs(2);redTrackIDs(2)];
    
    for i = 1:3
        text(h(1).XData(i) + h(1).XOffset, h(1).YData(i)/2, strcat('T',motTracks{i}),'HorizontalAlignment','center', 'VerticalAlignment','middle','FontSize',14,'Color',[1 1 1],'FontWeight','bold');
    end
    
    for i = 1:3
        text(h(2).XData(i) + h(2).XOffset, h(2).YData(i)/2, strcat('T',jpdaTracks{i}),'HorizontalAlignment','center', 'VerticalAlignment','middle','FontSize',14,'Color',[1 1 1],'FontWeight','bold');
    end
    
    ylabel('Number of Tracks');
    xlabel('Track Type');
    xticklabels({'Target Tracks','False Tracks','Redundant Tracks'});
    legend('Point Target Tracker GNN', 'Point Target Tracker JPDA');
end