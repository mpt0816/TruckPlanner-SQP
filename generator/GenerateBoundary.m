classdef GenerateBoundary < handle
    
    properties
        boundaries_
    end
    
    methods
        function obj = GenerateBoundary(scenario)
            obj.boundaries_.lane_boundary = obj.CalculateLaneBoundary(scenario.lane);
            obj.boundaries_.shoulder_boundary = obj.CalculateShoulderBoundary(scenario.lane);
            obj.boundaries_.obstacle_boundary = obj.CalculateObstacleBoundary(scenario.lane, scenario.obstacles);
        end
        
        function lane_boundary = CalculateLaneBoundary(obj, lane)
            config = GetPathPlannerConfig();
            buffer = config.lane_boundary_buffer;
            lower_bound = [];
            upper_bound = [];
            s = [];
            bound = lane.lane_width / 2.0;
            for pt = lane.center_line
                lower_bound = [lower_bound, -bound + buffer];
                upper_bound = [upper_bound, bound - buffer];
                s = [s, pt.s];
            end
            lane_boundary.lower_bound = lower_bound;
            lane_boundary.upper_bound = upper_bound;
            lane_boundary.s = s;
        end
        
        function shoulder_boundary = CalculateShoulderBoundary(obj, lane)
            config = GetPathPlannerConfig();
            params = GetTruckParams();
            chassis_height = params.chassis_height;
            buffer = config.shoulder_boundary_buffer;
            height_buffer = config.chassis_height_buffer;
            
            lane_bound = lane.lane_width / 2.0;
            left_shoulder_height = lane.left_shoulder_height;
            right_shoulder_height = lane.right_shoulder_height;
            left_shoulder_gap = lane.left_shoulder_gap;
            right_shoulder_gap = lane.right_shoulder_gap;
            
            if chassis_height - height_buffer <= left_shoulder_height
                left_bound = lane_bound + left_shoulder_gap - buffer;
            else
                left_bound = 10.0;
            end
            
            if chassis_height - height_buffer <= right_shoulder_height
                right_bound = lane_bound + right_shoulder_gap - buffer;
            else
                right_bound = 10.0;
            end
            
            lower_bound = [];
            upper_bound = [];
            s = [];
            for pt = lane.center_line
                lower_bound = [lower_bound, -right_bound];
                upper_bound = [upper_bound, left_bound];
                s = [s, pt.s];
            end
            shoulder_boundary.lower_bound = lower_bound;
            shoulder_boundary.upper_bound = upper_bound;
            shoulder_boundary.s = s;
        end
        
        function obstacle_boundary = CalculateObstacleBoundary(obj, lane, obstacles)
            config = GetPathPlannerConfig();
            buffer = config.obstacle_boundary_lat_buffer;
            
            lower_bound = [];
            upper_bound = [];
            s = [];
            
            if isempty(obstacles)
                for pt = lane.center_line
                    lower_bound = [lower_bound, -15.0];
                    upper_bound = [upper_bound, 15.0];
                    s = [s, pt.s];
                end
                obstacle_boundary.lower_bound = lower_bound;
                obstacle_boundary.upper_bound = upper_bound;
                obstacle_boundary.s = s;
                return;
            end
            
            sl_boundaries = [];
            for obstacle = obstacles
                sl_boundary = obj.CalculateObstacleSLBoundary(lane.center_line, obstacle);
                sl_boundaries = [sl_boundaries, sl_boundary];
            end
            
            num_of_obstacles = length(sl_boundaries);
            index_of_obstacle = 1;
            for pt = lane.center_line
                sl_boundary = sl_boundaries(index_of_obstacle);
                if pt.s < sl_boundary.s_min
                    lower_bound = [lower_bound, -15.0];
                    upper_bound = [upper_bound, 15.0];
                    s = [s, pt.s];
                elseif pt.s <= sl_boundary.s_max
                    if sl_boundary.l_min < 0.0 && sl_boundary.l_max < 0.0
                        lower_bound = [lower_bound, max(sl_boundary.l_min, sl_boundary.l_max) + buffer];
                        upper_bound = [upper_bound, 15.0];
                        s = [s, pt.s];
                    end
                    if sl_boundary.l_min > 0.0 && sl_boundary.l_max > 0.0
                        lower_bound = [lower_bound, -15.0];
                        upper_bound = [upper_bound, min(sl_boundary.l_min, sl_boundary.l_max) - buffer];
                        s = [s, pt.s];
                    end
                else
                    lower_bound = [lower_bound, -15.0];
                    upper_bound = [upper_bound, 15.0];
                    s = [s, pt.s];
                    if index_of_obstacle < num_of_obstacles
                        index_of_obstacle = index_of_obstacle + 1;
                    end
                end
            end
            obstacle_boundary.lower_bound = lower_bound;
            obstacle_boundary.upper_bound = upper_bound;
            obstacle_boundary.s = s;
        end
        
        function boundaries = GetBoundary(obj)
            boundaries = obj.boundaries_;
        end
        
        function sl_boundary = CalculateObstacleSLBoundary(obj, path, obstacle)
            corners = obstacle.corners;
            s = [];
            l = [];
            for corner = corners
                corner_sl = obj.CalculateSLPoint(path, corner);
                s = [s, corner_sl.s];
                l = [l, corner_sl.l];
            end
            
            sl_boundary.s_min = min(s);
            sl_boundary.s_max = max(s);
            sl_boundary.l_min = min(l);
            sl_boundary.l_max = max(l);
        end
        
        function sl_point = CalculateSLPoint(obj, path, point)
            min_dis_square = inf;
            for pt = path
                dis_square = DistanceSquare(pt, point);
                if dis_square < min_dis_square
                    min_dis_square = dis_square;
                    nearest_pt = pt;
                end
            end
            dx = point.x - nearest_pt.x;
            dy = point.y - nearest_pt.y;
            
            cos_theta_r = cos(nearest_pt.theta);
            sin_theta_r = sin(nearest_pt.theta);
            cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
            
            s = nearest_pt.s;
            l = sign(cross_rd_nd) * sqrt(dx * dx + dy * dy);
            
            sl_point.s = s;
            sl_point.l = l;
        end
        
        function value = LinearLnterpolation(obj, x0, y0, x1, y1, x)
            dx = x - x0;
            value = y0 + dx * (y1 - y0) / (x1 - x0);
        end
    end
end

