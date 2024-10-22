classdef GenerateLane < handle

    properties
        lane_width_ = 3.75
        resolution_ = 0.5
        left_shoulder_height_ = 0.15
        right_shoulder_height_ = 0.15
        left_shoulder_width_ = 3.0
        right_shoulder_width_ = 3.0
        left_shoulder_gap_ = 0.3
        right_shoulder_gap_ = 0.3
        kEpsilon = 1e-6
        spiral_path_generator_
    end
    
    methods
        function obj = GenerateLane()
            obj.spiral_path_generator_ = GenerateSpiralPath();
            obj.spiral_path_generator_.SetResolution(obj.resolution_);
        end
        
        function SetLaneWidth(obj, width)
            obj.lane_width_ = width;
        end
        
        function obj = SetResolution(obj, resolution)
            obj.resolution_ = resolution;
            obj.spiral_path_generator_.SetResolution(obj.resolution_);
        end
        
        function obj = SetShoulderWidth(obj, width)
            obj.left_shoulder_width_ = width;
            obj.right_shoulder_width_ = width;
        end
        
        function obj = SetLeftShoulderWidth(obj, width)
            obj.left_shoulder_width_ = width;
        end
        
        function obj = SetRightShoulderWidth(obj, width)
            obj.right_shoulder_width_ = width;
        end
        
        function obj = SetShoulderHeight(obj, height)
            obj.left_shoulder_height_ = height;
            obj.right_shoulder_height_ = height;
        end
        
        function obj = SetLeftShoulderHeight(obj, height)
            obj.left_shoulder_height_ = height;
        end
        
        function obj = SetRightShoulderHeight(obj, height)
            obj.right_shoulder_height_ = height;
        end
        
        function obj = SetShoulderGap(obj, gap)
            obj.right_shoulder_gap_ = gap;
            obj.left_shoulder_gap_ = gap;
        end
        
        function obj = SetLeftShoulderGap(obj, gap)
            obj.left_shoulder_gap_ = gap;
        end
        
        function obj = SetRightShoulderGap(obj, gap)
            obj.right_shoulder_gap_ = gap;
        end
        
        function lane = GenerateLaneWithTwoPoint(obj, start_pt, end_pt)
            lane.resolution = obj.resolution_;
            lane.lane_width = obj.lane_width_;
            lane.center_line = obj.spiral_path_generator_.GeneratePathWithTwoPoint(start_pt, end_pt);
            lane.left_boundary = obj.CalculateBoundary(lane.center_line, lane.lane_width / 2.0);
            lane.right_boundary = obj.CalculateBoundary(lane.center_line, -lane.lane_width / 2.0);
            
            lane.left_shoulder_height = obj.left_shoulder_height_;
            lane.left_shoulder_width = obj.left_shoulder_width_;
            lane.left_shoulder_gap = obj.left_shoulder_gap_;
            if lane.left_shoulder_height > obj.kEpsilon
                left_shoulder.right_boundary = obj.CalculateBoundary(lane.left_boundary, lane.left_shoulder_gap);
                left_shoulder.left_boundary = obj.CalculateBoundary(lane.left_boundary, lane.left_shoulder_gap + lane.left_shoulder_width);
            else
                left_shoulder.right_boundary = [];
                left_shoulder.left_boundary = [];
            end
            lane.left_shoulder = left_shoulder;
            
            lane.right_shoulder_height = obj.right_shoulder_height_;
            lane.right_shoulder_width = obj.right_shoulder_width_;
            lane.right_shoulder_gap = obj.right_shoulder_gap_;
            if lane.right_shoulder_height > obj.kEpsilon
                right_shoulder.left_boundary = obj.CalculateBoundary(lane.right_boundary, -lane.right_shoulder_gap);
                right_shoulder.right_boundary = obj.CalculateBoundary(lane.right_boundary, -lane.right_shoulder_gap - lane.right_shoulder_width);
            else
                right_shoulder.left_boundary = [];
                right_shoulder.right_boundary = [];
            end
            lane.right_shoulder = right_shoulder;
        end
        
        function lane = GenerateLaneWithPointList(obj, pts)
            lane.resolution = obj.resolution_;
            lane.lane_width = obj.lane_width_;
            lane.center_line = obj.spiral_path_generator_.GeneratePathWithPointList(pts);
            lane.left_boundary = obj.CalculateBoundary(lane.center_line, lane.lane_width / 2.0);
            lane.right_boundary = obj.CalculateBoundary(lane.center_line, -lane.lane_width / 2.0);
            
            lane.left_shoulder_height = obj.left_shoulder_height_;
            lane.left_shoulder_width = obj.left_shoulder_width_;
            lane.left_shoulder_gap = obj.left_shoulder_gap_;
            if lane.left_shoulder_height > obj.kEpsilon
                left_shoulder.right_boundary = obj.CalculateBoundary(lane.left_boundary, lane.left_shoulder_gap);
                left_shoulder.left_boundary = obj.CalculateBoundary(lane.left_boundary, lane.left_shoulder_gap + lane.left_shoulder_width);
            else
                left_shoulder.right_boundary = [];
                left_shoulder.left_boundary = [];
            end
            lane.left_shoulder = left_shoulder;
            
            lane.right_shoulder_height = obj.right_shoulder_height_;
            lane.right_shoulder_width = obj.right_shoulder_width_;
            lane.right_shoulder_gap = obj.right_shoulder_gap_;
            if lane.right_shoulder_height > obj.kEpsilon
                right_shoulder.left_boundary = obj.CalculateBoundary(lane.right_boundary, -lane.right_shoulder_gap);
                right_shoulder.right_boundary = obj.CalculateBoundary(lane.right_boundary, -lane.right_shoulder_gap - lane.right_shoulder_width);
            else
                right_shoulder.left_boundary = [];
                right_shoulder.right_boundary = [];
            end
            lane.right_shoulder = right_shoulder;
        end
        
        function boundary = CalculateBoundary(obj, center_line, offset)
            boundary = [];
            for center_pt = center_line
                boundary_pt = obj.CalculateBoundaryPoint(center_pt, offset);
                boundary = [boundary, boundary_pt];
            end
        end
        
        function point = CalculateBoundaryPoint(obj, center_point, offset)
            point = center_point;
            point.x = center_point.x - offset * sin(center_point.theta);
            point.y = center_point.y + offset * cos(center_point.theta);
        end
        
    end
end

