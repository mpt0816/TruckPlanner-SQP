classdef GenerateSpiralPath < handle
    properties
        resolution_ = 0.5
    end
    
    methods
        function obj = GenerateSpiralPath()
        end
        
        function obj = SetResolution(obj, resolution)
            obj.resolution_ = resolution;
        end
        
        function path = GeneratePathWithTwoPoint(obj, start_pt, end_pt)
            path = [];
            path = [path, start_pt];
            
            delta_s = end_pt.s - start_pt.s;
            angle_diff = AngleDiff(start_pt.theta, end_pt.theta);
            spiral_curve = QuinticSpiralCurve1d(start_pt.theta, start_pt.kappa, start_pt.dkappa, ...
                start_pt.theta + angle_diff, end_pt.kappa, end_pt.dkappa, delta_s);
            num_of_points = ceil(delta_s / obj.resolution_) + 1;
            
            for i = 1 : 1 : num_of_points
                inter_s = (delta_s / num_of_points) * i;
                dx = spiral_curve.ComputeCartesianDeviationX(inter_s);
                dy = spiral_curve.ComputeCartesianDeviationY(inter_s);
                theta = NormalizeAngle(spiral_curve.Evaluate(0, inter_s));
                kappa = spiral_curve.Evaluate(1, inter_s);
                dkappa = spiral_curve.Evaluate(2, inter_s);
                
                path_point = obj.ToPathPoint(start_pt.x + dx, start_pt.y + dy, start_pt.s + inter_s, ...
                    theta, kappa, dkappa);
                path = [path, path_point];
            end
            path = obj.RemoveDuplicates(path);
        end
        
        function path = GeneratePathWithPointList(obj, pts)
            path = [];
            num_of_pts = length(pts);
            for i = 1 : 1 : num_of_pts - 1
                start = pts(i);
                if ~isempty(path)
                    start = path(end);
                end
                segment = obj.GeneratePathWithTwoPoint(start, pts(i + 1));
                path = [path, segment];
            end
            path = obj.RemoveDuplicates(path);
        end
        
        function path_point = ToPathPoint(obj, x, y, s, theta, kappa, dkappa)
            path_point.x = x;
            path_point.y = y;
            path_point.s = s;
            path_point.theta = theta;
            path_point.kappa = kappa;
            path_point.dkappa = dkappa; 
        end
        
        function new_path = RemoveDuplicates(obj, path)
            kDuplicatedPointsEpsilon = 1e-6;
            limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
            new_path = [];
            for pt = path
                if isempty(new_path)
                    new_path = [new_path, pt];
                    continue;
                end
                s2 = (pt.s - new_path(end).s) * (pt.s - new_path(end).s);
                if DistanceSquare(pt, new_path(end)) > limit | s2 > limit
                    new_path = [new_path, pt];
                end
            end
        end
        
    end
end

