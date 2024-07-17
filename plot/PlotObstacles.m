function PlotObstacles(obstacles)

    if isempty(obstacles)
        return;
    end
    
    for obstacle = obstacles
        PlotObstacle(obstacle);
        hold on;
    end

end