function PlotFrenetProject(planned_sl_path, reference_line, boundaries)

num_of_pts = length(reference_line);

lane_boundar = boundaries.lane_boundary;
obstacle_boundary = boundaries.obstacle_boundary;

s_list = zeros(1, num_of_pts);
lower_bound = zeros(1, num_of_pts);
upper_bound = zeros(1, num_of_pts);

for i = 1 : 1 : num_of_pts
    pt = reference_line(i);
    s_list(1, i) = pt.s;
    lower_bound(1, i) = max(lane_boundar.lower_bound(i), obstacle_boundary.lower_bound(i));
    upper_bound(1, i) = min(lane_boundar.upper_bound(i), obstacle_boundary.upper_bound(i));
end

plot(s_list, lower_bound, 'r-', 'LineWidth', 1.0);
hold on;
p = plot(s_list, upper_bound, 'r-', 'LineWidth', 1.0);
set(p, 'handlevisibility', 'off');
hold on;


truck_param = GetTruckParams();
tractor_param = truck_param.tractor;
width_tractor = tractor_param.width / 2.0;
trailer_param = truck_param.trailer;
width_trailer = trailer_param.width / 2.0;


for i = 1 : 1 : num_of_pts
    s_tractor = [];
    l_tractor = [];
    s_trailer = [];
    l_trailer = [];
    
    %% Tractor 左前角点
    s = CalculateTractorStationByInIndex(i, tractor_param.wheel_base + tractor_param.front_oh, width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, tractor_param.wheel_base + tractor_param.front_oh, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 左前轴
    s = CalculateTractorStationByInIndex(i, tractor_param.wheel_base, width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, tractor_param.wheel_base, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 中心左侧
    s = CalculateTractorStationByInIndex(i, ...
        tractor_param.wheel_base + tractor_param.front_oh - tractor_param.length / 2.0, width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, ...
        tractor_param.wheel_base + tractor_param.front_oh - tractor_param.length / 2.0, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 左后轴
    s = CalculateTractorStationByInIndex(i, 0.0, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, 0.0, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 左后角点
    s = CalculateTractorStationByInIndex(i, -tractor_param.rear_oh, width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, -tractor_param.rear_oh, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 右后角点
    s = CalculateTractorStationByInIndex(i, -tractor_param.rear_oh, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, -tractor_param.rear_oh, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 右后轴
    s = CalculateTractorStationByInIndex(i, 0.0, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, 0.0, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 中心右侧
    s = CalculateTractorStationByInIndex(i, ...
        tractor_param.wheel_base + tractor_param.front_oh - tractor_param.length / 2.0, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, ...
        tractor_param.wheel_base + tractor_param.front_oh - tractor_param.length / 2.0, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 右前轴
    s = CalculateTractorStationByInIndex(i, tractor_param.wheel_base, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, tractor_param.wheel_base, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 右前角点
    s = CalculateTractorStationByInIndex(i, tractor_param.wheel_base + tractor_param.front_oh, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, tractor_param.wheel_base + tractor_param.front_oh, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    
    %% Trailer 左前角点
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge + trailer_param.front_oh, width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge + trailer_param.front_oh, width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 左前轴
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge, width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge, width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 中心左侧
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - (trailer_param.length / 2.0 - trailer_param.front_oh), width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - (trailer_param.length / 2.0 - trailer_param.front_oh), width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 左后轴
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base, width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base, width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 左后角点
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base - trailer_param.rear_oh, width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base - trailer_param.rear_oh, width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 右后角点
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base - trailer_param.rear_oh, -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base - trailer_param.rear_oh, -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 右后轴
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base, -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base, -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 中心右侧
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - (trailer_param.length / 2.0 - trailer_param.front_oh), -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - (trailer_param.length / 2.0 - trailer_param.front_oh), -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 右前轴
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge, -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge, -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 右前角点
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge + trailer_param.front_oh, -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge + trailer_param.front_oh, -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    
    tractor_color = [72 118 255] / 255;  %% RoyalBlue
    p1 = fill(s_tractor, l_tractor, ...
        tractor_color, 'facealpha', 0.1, 'edgealpha', 0);
    set(p1, 'handlevisibility', 'off');
    hold on;
    
    trailer_color = [255 215 0] / 255;  %% Gold
    p2 = fill(s_trailer, l_trailer, ...
        trailer_color, 'facealpha', 0.05, 'edgealpha', 0);
    set(p2, 'handlevisibility', 'off');
    hold on;
    
end

last_s = -inf;
for i = 1 : 1 : num_of_pts
    if reference_line(i).s - last_s < tractor_param.length + trailer_param.length && i < num_of_pts
        continue;
    end
    last_s = reference_line(i).s;
    
    s_tractor = [];
    l_tractor = [];
    s_trailer = [];
    l_trailer = [];
    
    %% Tractor 左前角点
    s = CalculateTractorStationByInIndex(i, tractor_param.wheel_base + tractor_param.front_oh, width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, tractor_param.wheel_base + tractor_param.front_oh, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 左前轴
    s = CalculateTractorStationByInIndex(i, tractor_param.wheel_base, width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, tractor_param.wheel_base, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 中心左侧
    s = CalculateTractorStationByInIndex(i, ...
        tractor_param.wheel_base + tractor_param.front_oh - tractor_param.length / 2.0, width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, ...
        tractor_param.wheel_base + tractor_param.front_oh - tractor_param.length / 2.0, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 左后轴
    s = CalculateTractorStationByInIndex(i, 0.0, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, 0.0, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 左后角点
    s = CalculateTractorStationByInIndex(i, -tractor_param.rear_oh, width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, -tractor_param.rear_oh, width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 右后角点
    s = CalculateTractorStationByInIndex(i, -tractor_param.rear_oh, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, -tractor_param.rear_oh, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 右后轴
    s = CalculateTractorStationByInIndex(i, 0.0, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, 0.0, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 中心右侧
    s = CalculateTractorStationByInIndex(i, ...
        tractor_param.wheel_base + tractor_param.front_oh - tractor_param.length / 2.0, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, ...
        tractor_param.wheel_base + tractor_param.front_oh - tractor_param.length / 2.0, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 右前轴
    s = CalculateTractorStationByInIndex(i, tractor_param.wheel_base, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, tractor_param.wheel_base, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    %% Tractor 右前角点
    s = CalculateTractorStationByInIndex(i, tractor_param.wheel_base + tractor_param.front_oh, -width_tractor);
    l = CalculateTractorEdgeLaterOffsetByIndex(i, tractor_param.wheel_base + tractor_param.front_oh, -width_tractor);
    s_tractor = [s_tractor, s];
    l_tractor = [l_tractor, l];
    
    
    %% Trailer 左前角点
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge + trailer_param.front_oh, width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge + trailer_param.front_oh, width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 左前轴
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge, width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge, width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 中心左侧
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - (trailer_param.length / 2.0 - trailer_param.front_oh), width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - (trailer_param.length / 2.0 - trailer_param.front_oh), width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 左后轴
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base, width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base, width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 左后角点
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base - trailer_param.rear_oh, width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base - trailer_param.rear_oh, width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 右后角点
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base - trailer_param.rear_oh, -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base - trailer_param.rear_oh, -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 右后轴
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base, -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - trailer_param.wheel_base, -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 中心右侧
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge - (trailer_param.length / 2.0 - trailer_param.front_oh), -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge - (trailer_param.length / 2.0 - trailer_param.front_oh), -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 右前轴
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge, -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge, -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    %% Trailer 右前角点
    s = CalculateTrailerStationByInIndex(i, ...
        -truck_param.tractor_base2hinge + trailer_param.front_oh, -width_trailer);
    l = CalculateTrailerEdgeLaterOffsetByIndex(i, ...
        -truck_param.tractor_base2hinge + trailer_param.front_oh, -width_trailer);
    s_trailer = [s_trailer, s];
    l_trailer = [l_trailer, l];
    
    
    tractor_color = [72 118 255] / 255;  %% RoyalBlue
    p1 = fill(s_tractor, l_tractor, ...
        tractor_color, 'facealpha', 0.8, 'edgealpha', 0.8);
    if i < num_of_pts
        set(p1, 'handlevisibility', 'off');
    end
    hold on;
    
    trailer_color = [255 215 0] / 255;  %% Gold
    p2 = fill(s_trailer, l_trailer, ...
        trailer_color, 'facealpha', 0.8, 'edgealpha', 0.8);
    if i < num_of_pts
        set(p2, 'handlevisibility', 'off');
    end
    hold on;
    
    p1 = scatter(s_tractor, l_tractor, 1,'MarkerEdgeColor',[0 .5 .5],...
        'MarkerFaceColor',[0 .7 .7],'LineWidth',1.5);
    set(p1, 'handlevisibility', 'off');
    hold on;
    p2 = scatter(s_trailer, l_trailer, 1,'MarkerEdgeColor',[0 .5 .5],...
        'MarkerFaceColor',[0 .7 .7], 'LineWidth',1.5);
    set(p2, 'handlevisibility', 'off');
    hold on;
    
    if i == num_of_pts
        p1 = scatter(s_tractor(1:1), l_tractor(1:1), 1,'MarkerEdgeColor',[0 .5 .5],...
            'MarkerFaceColor',[0 .7 .7],'LineWidth',1.5);
        hold on;
    end
end

set(gca, 'XLim', [s_list(1, 1) - 20.0, s_list(1, end) + 20]);
set(gca, 'YLim', [min(lower_bound) - 1.0, max(upper_bound) + 2.0]);
xlabel('s(m)');
ylabel('offset(m)');
legend({'Boundary', 'Tractor Body Project', 'Trailer Body Project', 'Project Point'}, ...
    'Location','northeast', 'FontSize', 8);

    function offset = CalculateTractorEdgeLaterOffsetByIndex(index, length, width)
        kEpsilon = 1e-6;
        road_kappa = CalculateRoadKappaByIndex(index);
        road_radius = 1.0 / (road_kappa + kEpsilon);
        
        if road_kappa >= -kEpsilon
            edge_radius = abs(road_radius) + width;
            K = 1.0;
        else
            edge_radius = abs(road_radius) - width;
            K = -1.0;
        end
        
        linearization_reference_pt = GetLinearizationReferencePoint(index);
        error_y_bar = linearization_reference_pt.error_y;
        error_theta_bar = linearization_reference_pt.error_theta;
        beta_bar = linearization_reference_pt.beta;
        kappa_bar = linearization_reference_pt.kappa;
        
        temp = length * cos(error_theta_bar) - width * sin(error_theta_bar) - road_radius * sin(error_theta_bar);
        
        offset = error_y_bar - road_radius * cos(error_theta_bar) + ...
            K * sqrt(edge_radius^2 - temp^2);
    end

    function offset = CalculateTrailerEdgeLaterOffsetByIndex(index, length, width)
        kEpsilon = 1e-6;
        road_kappa = CalculateRoadKappaByIndex(index);
        road_radius = 1.0 / (road_kappa + kEpsilon);
        
        if road_kappa >= -kEpsilon
            edge_radius = abs(road_radius) + width;
            K = 1.0;
        else
            edge_radius = abs(road_radius) - width;
            K = -1.0;
        end
        
        linearization_reference_pt = GetLinearizationReferencePoint(index);
        error_y_bar = linearization_reference_pt.error_y;
        error_theta_bar = linearization_reference_pt.error_theta;
        beta_bar = linearization_reference_pt.beta;
        kappa_bar = linearization_reference_pt.kappa;
        
        %             arc_length = length * cos(error_theta_bar + beta_bar) - width * sin(error_theta_bar + beta_bar);
        %             alpha = arc_length / abs(road_radius);
        %             delta_s = edge_radius * sin(alpha);
        %
        %             temp = delta_s - road_radius * sin(error_theta_bar + beta_bar);
        %
        %             offset = error_y_bar - road_radius * cos(error_theta_bar + beta_bar) + ...
        %                 K * sqrt(edge_radius^2 - temp^2);
        
        temp = length * cos(error_theta_bar + beta_bar) - ...
            width * sin(error_theta_bar + beta_bar) - ...
            road_radius * sin(error_theta_bar + beta_bar);
        
        offset = error_y_bar - road_radius * cos(error_theta_bar + beta_bar) + ...
            K * sqrt(edge_radius^2 - temp^2);
    end

    function road_kappa = CalculateRoadKappaByIndex(index)
        road_pt = reference_line(index);
        road_kappa = road_pt.kappa;
    end

    function linearization_reference_pt = GetLinearizationReferencePoint(index)
        linearization_reference_pt = planned_sl_path(index);
    end

    function s_hat = CalculateTractorStationByInIndex(index, length, width)
        ref_pt = reference_line(index);
        ref_s = ref_pt.s;
        
        linearization_reference_pt = GetLinearizationReferencePoint(index);
        error_theta_bar = linearization_reference_pt.error_theta;
        s_hat = ref_s + length * cos(error_theta_bar) - width * sin(error_theta_bar);
    end

    function s_hat = CalculateTrailerStationByInIndex(index, length, width)
        ref_pt = reference_line(index);
        ref_s = ref_pt.s;
        
        linearization_reference_pt = GetLinearizationReferencePoint(index);
        error_theta_bar = linearization_reference_pt.error_theta;
        beta_bar = linearization_reference_pt.beta;
        
        s_hat = ref_s + length * cos(error_theta_bar + beta_bar) - width * sin(error_theta_bar + beta_bar);
    end

end