function PlotBoundary(boundary, marker)
    lower_bound = boundary.lower_bound;
    upper_bound = boundary.upper_bound;
    s = boundary.s;
    if isempty(lower_bound) | isempty(upper_bound)
        return;
    end
    
    p_lower = plot(s, lower_bound, marker, 'LineWidth', 1.0);
    hold on;
    p_upper = plot(s, upper_bound, marker, 'LineWidth', 1.0);
    set(p_upper, 'handlevisibility', 'off');
end

