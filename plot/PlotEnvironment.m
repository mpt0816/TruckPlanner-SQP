function region = PlotEnvironment(scenario)
    region = PlotLane(scenario.lane);
    hold on;
    PlotObstacles(scenario.obstacles);
end