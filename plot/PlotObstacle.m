function PlotObstacle(obstacle)
    corners = obstacle.corners;
    x_list = [];
    y_list = [];
    for corner = corners
        x_list = [x_list, corner.x];
        y_list = [y_list, corner.y];
    end
    obstacle_color = [205 51 51] / 255;  %% Brown;
    p_obstalce = fill(x_list, y_list, obstacle_color);
    set(p_obstalce, 'handlevisibility', 'off');
end