function config = GetPathPlannerConfig()
    config.lane_boundary_buffer = 0.1;
    config.shoulder_boundary_buffer = 0.1;
    config.obstacle_boundary_lat_buffer = 0.1;
    config.obstacle_boundary_lon_buffer = 0.1;
    config.chassis_height_buffer = 0.1;
    
    config.max_kappa = 0.1;
    config.max_dkappa = 0.1 / 10;
    
    config.offset_ratio = 0.6;
    config.weight_offset = 1.0;
    config.weight_theta = 10.0;
    config.weight_beta = 0.0;
    config.weight_kappa = 0.0;
    config.weight_dkappa = 100000.0;
    config.weight_comfort = 10.0;
end