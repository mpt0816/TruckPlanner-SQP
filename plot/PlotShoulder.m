function region = PlotShoulder(shoulder)
    left_boundary = shoulder.left_boundary;
    right_boundary = shoulder.right_boundary;
    
    if isempty(left_boundary) || isempty(right_boundary)
        return;
    end
    
    left_boundary_x = [];
    left_boundary_y = [];
    
    right_boundary_x = [];
    right_boundary_y = [];
    
    for pt = left_boundary
        left_boundary_x = [left_boundary_x, pt.x];
        left_boundary_y = [left_boundary_y, pt.y];
    end
    
    for pt = right_boundary
        right_boundary_x = [right_boundary_x, pt.x];
        right_boundary_y = [right_boundary_y, pt.y];
    end
    
    region.min_x = min([min(left_boundary_x), min(right_boundary_x)]);
    region.max_x = max([max(left_boundary_x), max(right_boundary_x)]);
    region.min_y = min([min(left_boundary_y), min(right_boundary_y)]);
    region.max_y = max([max(left_boundary_y), max(right_boundary_y)]);
    
    shoulder_color = [105 105 105] / 255; %% grey
    h = fill([left_boundary_x, flip(right_boundary_x)], [left_boundary_y, flip(right_boundary_y)], shoulder_color);
    set(h,'edgealpha',0);
    set(h, 'handlevisibility', 'off');

end