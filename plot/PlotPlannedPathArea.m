function PlotPlannedPathArea(path, facealpha, edgealpha)
    

    for pt = path
        PlotTruckArea(pt, false, facealpha, edgealpha);
        
    end
    
end