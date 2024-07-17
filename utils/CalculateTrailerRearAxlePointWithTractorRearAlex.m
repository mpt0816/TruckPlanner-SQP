function pt_rear_trailer = CalculateTrailerRearAxlePointWithTractorRearAlex(pt_rear_tractor)
    pt_rear_trailer = pt_rear_tractor;
    params = GetTruckParams();
    wheel_base_trailer = params.trailer.wheel_base;
    
    pt_hinge = CalculatePointByHeading(pt_rear_tractor, params.tractor_base2hinge);
    
    pt_rear_trailer.theta = NormalizeAngle(pt_hinge.theta + pt_hinge.beta);
    pt_rear_trailer.x = pt_hinge.x - wheel_base_trailer * cos(pt_rear_trailer.theta);
    pt_rear_trailer.y = pt_hinge.y - wheel_base_trailer * sin(pt_rear_trailer.theta);
end