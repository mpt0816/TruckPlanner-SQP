function PlotTrajectoryLine(path, trailer_visible, handlevisibility, color_locked)
    x_tractor = [];
    y_tractor = [];
    x_trailer = [];
    y_trailer = [];
    for pt = path
        x_tractor = [x_tractor, pt.x];
        y_tractor = [y_tractor, pt.y];
        
        pt_trailer = CalculateTrailerRearAxlePointWithTractorRearAlex(pt);
        x_trailer = [x_trailer, pt_trailer.x];
        y_trailer = [y_trailer, pt_trailer.y];
    end

    if color_locked
        tractor_color = [72 118 255] / 255;  %% RoyalBlue
        p1 = plot(x_tractor, y_tractor, 'Color', tractor_color, 'LineWidth', 0.5);
    else
        p1 = plot(x_tractor, y_tractor, 'LineWidth', 1.0);
    end
    
    if ~handlevisibility 
        set(p1, 'handlevisibility', 'off'); 
    end
    hold on;
    
    if ~trailer_visible
        return;
    end
    
    if color_locked
        trailer_color = [255 215 0] / 255;  %% Gold
        p2 = plot(x_trailer, y_trailer, 'Color', trailer_color, 'LineWidth', 0.5);
    else
        p2 = plot(x_trailer, y_trailer, 'LineWidth', 0.5);
    end
    
    if ~handlevisibility 
        set(p2, 'handlevisibility', 'off'); 
    end
    hold on;
end