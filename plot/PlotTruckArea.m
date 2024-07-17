function PlotTruckArea(planned_path_point, handlevisibility, facealpha, edgealpha)
    params = GetTruckParams();
    tractor = params.tractor;
    trailer = params.trailer;

    pt_rear_tractor = planned_path_point;
    pt_rear_trailer = CalculateTrailerRearAxlePointWithTractorRearAlex(pt_rear_tractor);

    tractor_coners = CalculateConersFromRearPoint(pt_rear_tractor, tractor);
    trailer_coners = CalculateConersFromRearPoint(pt_rear_trailer, trailer);

    tractor_color = [72 118 255] / 255;  %% RoyalBlue
    p1 = fill([tractor_coners(1).x, tractor_coners(2).x, tractor_coners(3).x, tractor_coners(4).x], ...
            [tractor_coners(1).y, tractor_coners(2).y, tractor_coners(3).y, tractor_coners(4).y], tractor_color, 'facealpha', facealpha);
    if ~handlevisibility 
        set(p1, 'handlevisibility', 'off'); 
    end
    
    set(p1, 'edgealpha', edgealpha);
    hold on;

    trailer_color = [255 215 0] / 255;  %% Gold
    p2 = fill([trailer_coners(1).x, trailer_coners(2).x, trailer_coners(3).x, trailer_coners(4).x], ...
            [trailer_coners(1).y, trailer_coners(2).y, trailer_coners(3).y, trailer_coners(4).y], trailer_color, 'facealpha', facealpha);
    if ~handlevisibility 
        set(p2, 'handlevisibility', 'off'); 
    end
    set(p2, 'edgealpha', edgealpha);
    hold on;
end