function PlotBoundaries(boundaries)
    PlotBoundary(boundaries.lane_boundary, 'b-');
    hold on;
    
%     PlotBoundary(boundaries.shoulder_boundary, 'k-');
%     hold on;
    
    PlotBoundary(boundaries.obstacle_boundary, 'r-');
    xlabel('s(m)');
    ylabel('l(m)');
    axis equal;
    legend('lane boundary',  'obstacle boundary');
end

