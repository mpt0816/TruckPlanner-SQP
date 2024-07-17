classdef GenerateObstacle < handle
    
    properties
        width_ = 2.2
        length_ = 4.5
        type_ = 'car'
        num_ = 1
        invade_ = 0.0
        lane_
        obstacles_ = [];
        kEpsilon = 1e-6
    end
    
    methods
        function obj = GenerateObstacle(lane, type, num, invade)
            obj.lane_ = lane;
            obj.type_ = type;
            obj.num_ = num;
            obj.invade_ = invade;
            switch(obj.type_)
                case 'car'
                    obj.width_ = 2.2;
                    obj.length_ = 4.5;
                case 'bus'
                    obj.width_ = 2.5;
                    obj.length_ = 12.5;
                otherwise
                    obj.width_ = 2.2;
                    obj.length_ = 4.5;
            end
            obj.obstacles_ = obj.ObstaclesMethod2();
        end
        
        function obstacles = Obstacles(obj)
            lane_length = obj.lane_.center_line(end).s;
            delta_s = lane_length / (obj.num_ + 1);
            
            obstacles = [];
            if obj.num_ < obj.kEpsilon
                return
            end
            
            sign_coef = 1;
            for i = 1 : 1 : obj.num_
                s = delta_s * i;
                for pt = obj.lane_.center_line
                    if pt.s >= s
                        center_line_pt = pt;
                        break;
                    end
                end
                offset = obj.lane_.lane_width / 2.0 - obj.invade_  + obj.width_ / 2.0;
                offset = sign_coef * offset;
                offset = obj.AmendOffsetByKappa(offset, center_line_pt.kappa);
                obs_center_pt = obj.PointOffset(center_line_pt, offset);
                obstacle = obj.Obstacle(obs_center_pt);
                obstacles = [obstacles, obstacle];
                sign_coef = -1 * sign_coef;
            end  
        end
        
        function obstacle = Obstacle(obj, center_pt)
            half_length = obj.length_ / 2.0;
            half_width = obj.width_ / 2.0;
            corners = [];
            pt1.x = center_pt.x + half_length * cos(center_pt.theta) - half_width * sin(center_pt.theta);
            pt1.y = center_pt.y + half_length * sin(center_pt.theta) + half_width * cos(center_pt.theta);
            corners = [corners, pt1]; 
            
            pt2.x = center_pt.x - half_length * cos(center_pt.theta) - half_width * sin(center_pt.theta);
            pt2.y = center_pt.y - half_length * sin(center_pt.theta) + half_width * cos(center_pt.theta);
            
            for ratio = 0.1 : 0.1 : 0.9
                pt = obj.LinearLnterpolation(pt1, pt2, ratio);
                corners = [corners, pt]; 
            end
            corners = [corners, pt2];
            
            pt3.x = center_pt.x - half_length * cos(center_pt.theta) + half_width * sin(center_pt.theta);
            pt3.y = center_pt.y - half_length * sin(center_pt.theta) - half_width * cos(center_pt.theta);
            
            for ratio = 0.1 : 0.1 : 0.9
                pt = obj.LinearLnterpolation(pt2, pt3, ratio);
                corners = [corners, pt]; 
            end
            corners = [corners, pt3];
            
            pt4.x = center_pt.x + half_length * cos(center_pt.theta) + half_width * sin(center_pt.theta);
            pt4.y = center_pt.y + half_length * sin(center_pt.theta) - half_width * cos(center_pt.theta);
           
            for ratio = 0.1 : 0.1 : 0.9
                pt = obj.LinearLnterpolation(pt3, pt4, ratio);
                corners = [corners, pt]; 
            end
            corners = [corners, pt4];
            
            for ratio = 0.1 : 0.1 : 0.9
                pt = obj.LinearLnterpolation(pt4, pt1, ratio);
                corners = [corners, pt]; 
            end
            
            obstacle.corners = corners;
            obstacle.type = obj.type_;
            obstacle.width = obj.width_;
            obstacle.length = obj.length_;
        end
        
        function obstacles = GetObstacles(obj)
            obstacles = obj.obstacles_;
        end
        
        function point = PointOffset(obj, center_point, offset)
            point = center_point;
            point.x = center_point.x - offset * sin(center_point.theta);
            point.y = center_point.y + offset * cos(center_point.theta);
        end
        
        function point = LinearLnterpolation(obj, pt1, pt2, ratio)
            point.x = pt1.x + ratio * (pt2.x - pt1.x);
            point.y = pt1.y + ratio * (pt2.y - pt1.y);
        end
        
        function new_offset = AmendOffsetByKappa(obj, offset, kappa)
            config = GetPathPlannerConfig();
            max_kappa = 0.3 * config.max_kappa;
            new_offset = offset;
            
            width = obj.lane_.lane_width / 2.0;
            if abs(offset) + 0.2 > width
                return
            end
            
            kappa_abs = abs(kappa);
            if kappa_abs > max_kappa
                new_offset = (1.0 + min(1.0, (kappa_abs - max_kappa) / kappa_abs)) * offset;
            end
        end
        
        function obstacles = ObstaclesMethod2(obj)
            lane_length = obj.lane_.center_line(end).s;
            delta_s = lane_length / (obj.num_ + 1);
            
            obstacles = [];
            if obj.num_ < obj.kEpsilon
                return
            end
            
            sign_coef = 1;
            for i = 1 : 1 : obj.num_
                s = delta_s * i;
                for pt = obj.lane_.center_line
                    if pt.s >= s
                        center_line_pt = pt;
                        break;
                    end
                end
                offset = sign_coef * obj.invade_;
                obstacle = obj.ObstacleMethod2(center_line_pt.s, offset);
                obstacles = [obstacles, obstacle];
                sign_coef = -1 * sign_coef;
            end  
        end
        
        function obstacle = ObstacleMethod2(obj, s, offset)
            buffer = 0.3;
            width = obj.lane_.lane_width / 2.0 - abs(offset) + buffer;
            if abs(offset) >= obj.lane_.lane_width / 2.0
                width = obj.width_;
            end
            corners_1 = [];
            corners_2 = [];
            for pt = obj.lane_.center_line
                if pt.s < s
                    continue;
                end
                if pt.s > s + obj.length_
                    break;
                end
                corner_1 = obj.PointOffset(pt, offset);
                corner_2 = obj.PointOffset(pt, offset + sign(offset) * width);
                corners_1 = [corners_1, corner_1];
                corners_2 = [corners_2, corner_2];
            end
            
            obstacle.corners = [corners_1, flip(corners_2)];
            obstacle.type = obj.type_;
            obstacle.width = width;
            obstacle.length = obj.length_;
        end
        
    end
end

