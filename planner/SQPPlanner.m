classdef SQPPlanner < handle
    
    properties
        reference_line_
        lane_boundary_
        obstacle_boundary_
        delta_s_
        planned_sl_result_
        planned_path_
        iter_path_
        linearization_reference_
        num_of_knots_
        tolerance_abs_ = 1e-2
        tolerance_rel_ = 1e-2
        qp_iter_ = 5000;
        sqp_iter_ = 5;
        kEpsilon = 1e-6
        
        osqp_warm_start_x_ = [];
        osqp_warm_start_y_ = [];
    end
    
    methods
        function obj = SQPPlanner(lane, boundaries, init_linearization_reference)
            obj.reference_line_ = lane.center_line;
            obj.lane_boundary_ = boundaries.lane_boundary;
            obj.obstacle_boundary_ = boundaries.obstacle_boundary;
            obj.delta_s_ = lane.resolution;
            obj.linearization_reference_ = init_linearization_reference;
            obj.planned_sl_result_ = init_linearization_reference;
            obj.num_of_knots_ = length(obj.reference_line_);
        end
        
        function path = GetPlannedPath(obj)
            path = obj.planned_path_;
        end
        
        function path = GetIterPlannedPath(obj)
            path = obj.iter_path_;
        end
        
        function sl_path = GetPlannedSLPath(obj)
            sl_path = obj.planned_sl_result_;
        end
        
        function out = SolveSQPProblem(obj)
            iter = 1;
            while iter <= obj.sqp_iter_
                obj.SetLinearizationReference();
                qp_problem = obj.FormulateQPProblem();
                qp_results = obj.SolveQPProblem(qp_problem);
                
                if qp_results.info.status_val < 0 || (~(qp_results.info.status_val == 1) && ~(qp_results.info.status_val ==2))
                    out = false;
                    %% 32是空格
                    disp(strcat('Osqp Failed in ', 32, num2str(iter), 'th iter!'));
                    return;
                end
                
                obj.DataTransform(qp_results);
                tolerance = obj.CalculateTolerance();
                iter_path.iter = iter;
                iter_path.path = obj.planned_path_;
                
                iter_path.max_offset_eps_abs = tolerance.max_y_error_abs;
                iter_path.max_theta_eps_abs = tolerance.max_theta_error_abs;
                iter_path.max_beta_eps_abs = tolerance.max_beta_error_abs;
                iter_path.max_kappa_eps_abs = tolerance.max_kappa_error_abs;
                
                iter_path.max_offset_eps_rel = tolerance.max_y_error_rel;
                iter_path.max_theta_eps_rel = tolerance.max_theta_error_rel;
                iter_path.max_beta_eps_rel = tolerance.max_beta_error_rel;
                iter_path.max_kappa_eps_rel = tolerance.max_kappa_error_rel;
                
                obj.iter_path_ = [obj.iter_path_, iter_path];
                if obj.IsConvergence(tolerance)
                    out = true;
                    %% 32是空格
                    disp(strcat('SQP Convergence in ', 32, num2str(iter), 'th iter!'));
                    return;
                end
                iter = iter + 1;
            end
            disp(strcat('SQP Convergence in ', 32, num2str(iter - 1), 'th iter!'));
            out = true;
        end
        
        function out = IsConvergence(obj, tolerance)
            
            if tolerance.max_y_error_abs > obj.tolerance_abs_ || ...
                    tolerance.max_theta_error_abs > obj.tolerance_abs_ || ...
                    tolerance.max_beta_error_abs > obj.tolerance_abs_ || ...
                    tolerance.max_kappa_error_abs > obj.tolerance_abs_
                out = false;
                return;
            end
            
            if max([tolerance.max_y_error_rel, tolerance.max_theta_error_rel, ...
                    tolerance.max_beta_error_rel, tolerance.max_kappa_error_rel]) > obj.tolerance_rel_
                out = false;
                return;
            end
            out = true;
        end
        
        function tolerance = CalculateTolerance(obj)
            tolerance.max_y_error_abs = -inf;
            tolerance.max_y_error_rel = -inf;
            tolerance.max_theta_error_abs = -inf;
            tolerance.max_theta_error_rel = -inf;
            tolerance.max_beta_error_abs = -inf;
            tolerance.max_beta_error_rel = -inf;
            tolerance.max_kappa_error_abs = -inf;
            tolerance.max_kappa_error_rel = -inf;
            
            for i = 1 : 1 : obj.num_of_knots_
                y_error_abs = abs(obj.linearization_reference_(i).error_y - ...
                    obj.planned_sl_result_(i).error_y);
                y_error_rel = y_error_abs / abs(obj.linearization_reference_(i).error_y + obj.kEpsilon);
                if y_error_abs > tolerance.max_y_error_abs
                    tolerance.max_y_error_abs = y_error_abs;
                end
                if y_error_rel > tolerance.max_y_error_rel
                    tolerance.max_y_error_rel = y_error_rel;
                end
                
                theta_error_abs = abs(obj.linearization_reference_(i).error_theta - ...
                    obj.planned_sl_result_(i).error_theta);
                theta_error_rel = theta_error_abs / abs(obj.linearization_reference_(i).error_theta + obj.kEpsilon);
                if theta_error_abs > tolerance.max_theta_error_abs
                    tolerance.max_theta_error_abs = theta_error_abs;
                end
                if theta_error_rel > tolerance.max_theta_error_rel
                    tolerance.max_theta_error_rel = theta_error_rel;
                end
                
                beta_error_abs = abs(obj.linearization_reference_(i).beta - ...
                    obj.planned_sl_result_(i).beta);
                beta_error_rel = beta_error_abs / abs(obj.linearization_reference_(i).beta + obj.kEpsilon);
                if beta_error_abs > tolerance.max_beta_error_abs
                    tolerance.max_beta_error_abs = beta_error_abs;
                end
                if beta_error_rel > tolerance.max_beta_error_rel
                    tolerance.max_beta_error_rel = beta_error_rel;
                end
                
                kappa_error_abs = abs(obj.linearization_reference_(i).kappa - ...
                    obj.planned_sl_result_(i).kappa);
                kappa_error_rel = kappa_error_abs / abs(obj.linearization_reference_(i).kappa + obj.kEpsilon);
                if kappa_error_abs > tolerance.max_kappa_error_abs
                    tolerance.max_kappa_error_abs = kappa_error_abs;
                end
                if kappa_error_rel > tolerance.max_kappa_error_rel
                    tolerance.max_kappa_error_rel = kappa_error_rel;
                end
                
            end
        end
        
        function obj = SetLinearizationReference(obj)
            obj.linearization_reference_ = obj.planned_sl_result_;
        end
        
        function obj = DataTransform(obj, qp_result)
            for i = 1 : 1 : obj.num_of_knots_ - 1
                obj.planned_sl_result_(i).error_y = qp_result.x(3 * i - 2);
                obj.planned_sl_result_(i).error_theta = NormalizeAngle(qp_result.x(3 * i - 1));
                obj.planned_sl_result_(i).beta = NormalizeAngle(qp_result.x(3 * i));
                obj.planned_sl_result_(i).kappa = qp_result.x(3 * obj.num_of_knots_ + i);
            end
            obj.planned_sl_result_(obj.num_of_knots_).error_y = qp_result.x(3 * obj.num_of_knots_ - 2);
            obj.planned_sl_result_(obj.num_of_knots_).error_theta = NormalizeAngle(qp_result.x(3 * obj.num_of_knots_ - 1));
            obj.planned_sl_result_(obj.num_of_knots_).beta = NormalizeAngle(qp_result.x(3 * obj.num_of_knots_));
            obj.planned_sl_result_(obj.num_of_knots_).kappa = qp_result.x(4 * obj.num_of_knots_ - 1);
            for i = 1 : 1 : obj.num_of_knots_ - 2
                obj.planned_sl_result_(i).dkappa = (obj.planned_sl_result_(i + 1).kappa - obj.planned_sl_result_(i).kappa) / obj.delta_s_;
            end
            obj.planned_sl_result_(obj.num_of_knots_ - 1).dkappa = obj.planned_sl_result_(obj.num_of_knots_ - 2).dkappa;
            obj.planned_sl_result_(obj.num_of_knots_).dkappa = obj.planned_sl_result_(obj.num_of_knots_ - 1).dkappa;
            obj.planned_path_ = obj.ConvertSLPathToCartesian(obj.planned_sl_result_);
        end
        
        function path = ConvertSLPathToCartesian(obj, sl_path)
            num = length(sl_path);
            path = repmat(struct('x', 0, 'y', 0, 's', 0, 'theta', 0, 'beta', 0, 'kappa', 0, 'dkappa', 0), 1, num);
            for i = 1 : 1 : num
                sl_pt = sl_path(i);
                ref_pt = obj.reference_line_(i);
                path(i) = obj.ConvertSLPointToCartesian(sl_pt, ref_pt);
            end
            path = obj.UpdateStation(path);
        end
        
        function point = ConvertSLPointToCartesian(obj, sl_pt, ref_pt)
            point.x = ref_pt.x - sl_pt.error_y * sin(ref_pt.theta);
            point.y = ref_pt.y + sl_pt.error_y * cos(ref_pt.theta);
            point.s = ref_pt.s;
            point.theta = NormalizeAngle(ref_pt.theta + sl_pt.error_theta);
            point.beta = NormalizeAngle(sl_pt.beta);
            point.kappa = sl_pt.kappa;
            point.dkappa = sl_pt.dkappa;
        end
        
        function new_path = UpdateStation(obj, path)
            new_path = path;
            num = length(path);
            for i = 1 : 1 : num
                if i == 1
                    path(i).s = 0.0;
                else
                    dx = path(i).x - path(i - 1).x;
                    dy = path(i).y - path(i - 1).y;
                    path(i).s = path(i - 1).s + sqrt(dx * dx + dy * dy);
                end
            end
        end
        
        function problem = FormulateQPProblem(obj)
            kernel = obj.CalcaulateKernel();
            problem.H = kernel.H;
            problem.G = kernel.G;
            constraint = obj.CalculateConstraint();
            problem.A = constraint.A;
            problem.lower = constraint.lower;
            problem.upper = constraint.upper;
        end
        
        function results = SolveQPProblem(obj, problem)
            P = sparse(problem.H);
            q = problem.G;
            A = sparse(problem.A);
            l = problem.lower;
            u = problem.upper;
            
            solver = osqp;
            settings = obj.OsqpSettings(solver);
            solver.setup(P, q, A, l, u, settings);
            solver = obj.OsqpWarmStartSet(solver);
            results = solver.solve();
            
            obj.osqp_warm_start_x_ = results.x;
            obj.osqp_warm_start_y_ = results.y;
        end
        
        function settings = OsqpSettings(obj, solver)
            settings = solver.default_settings();
            settings.max_iter = obj.qp_iter_;
            settings.polish = true;
            settings.verbose = false;
            settings.scaled_termination = false;
            settings.warm_start = true;
        end
        
        function solver = OsqpWarmStartSet(obj, osqp_solver)
            
            if isempty(obj.osqp_warm_start_x_)
                obj.osqp_warm_start_x_ = zeros(1, 4 * obj.num_of_knots_ -  1);
                for i = 1 : 1 : obj.num_of_knots_ - 1
                    obj.osqp_warm_start_x_(1, 3 * i - 2) = obj.planned_sl_result_(i).error_y;
                    obj.osqp_warm_start_x_(1, 3 * i - 1) = obj.planned_sl_result_(i).error_theta;
                    obj.osqp_warm_start_x_(1, 3 * i) = obj.planned_sl_result_(i).beta;
                    obj.osqp_warm_start_x_(1, 3 * obj.num_of_knots_ + i) = obj.planned_sl_result_(i).kappa;
                end
                obj.osqp_warm_start_x_(1, 3 * obj.num_of_knots_ - 2) = obj.planned_sl_result_(obj.num_of_knots_).error_y;
                obj.osqp_warm_start_x_(1, 3 * obj.num_of_knots_ - 1) = obj.planned_sl_result_(obj.num_of_knots_).error_theta;
                obj.osqp_warm_start_x_(1, 3 * obj.num_of_knots_) = obj.planned_sl_result_(obj.num_of_knots_).beta;
            end
            if isempty(obj.osqp_warm_start_y_)
                osqp_solver.warm_start('x', obj.osqp_warm_start_x_);
            else
                osqp_solver.warm_start('x', obj.osqp_warm_start_x_, 'y', obj.osqp_warm_start_y_);
            end
            
            solver = osqp_solver;
        end
        
        function kernel = CalcaulateKernel(obj)
            config = GetPathPlannerConfig();
            weight_kappa = config.weight_kappa;
            weight_dkappa = config.weight_dkappa;
            
            truck_param = GetTruckParams();
            trailer_param = truck_param.trailer;
            length = trailer_param.wheel_base;
            width = 0.0;
            
            kernel_dim = 4 * obj.num_of_knots_ - 1;
            kernel.H = zeros(kernel_dim, kernel_dim);
            kernel.G = zeros(kernel_dim, 1);
            %% 状态变量的权重
            for i = 1 : 1 : obj.num_of_knots_
                road_pt = obj.reference_line_(i);
                road_kappa = road_pt.kappa;
                best_k = obj.CalculateBestLaterOffsetParam(road_kappa);
                coeff = obj.CalculateTrailerEdgeLaterOffsetByIndex(i, -length, width);
                kernel.H(3 * (i - 1) + 1 : 3 * i, 3 * (i - 1) + 1 : 3 * i) = obj.FormulateMatrix(best_k, coeff).H;
                %                 kernel.H(3 * (i - 1) + 1 : 3 * i, 3 * (i - 1) + 1 : 3 * i) = [1.0, 0.0, 0.0; 0.0, 0.0, 0.0; 0.0, 0.0, 0.0];
                kernel.G(3 * (i - 1) + 1 : 3 * i, 1) = obj.FormulateMatrix(best_k, coeff).G;
            end
            %% 曲率的权重
            kernel.H(3 * obj.num_of_knots_ + 1, 3 * obj.num_of_knots_ + 1 : 3 * obj.num_of_knots_ + 2) = [weight_kappa + weight_dkappa, -weight_dkappa];
            for i = 3 * obj.num_of_knots_ + 2 : 1 : kernel_dim - 1
                kernel.H(i, i - 1 : i + 1) = [-weight_dkappa, weight_kappa + 2 * weight_dkappa, -weight_dkappa];
            end
            kernel.H(kernel_dim, kernel_dim - 1 : kernel_dim) = [-weight_dkappa, weight_kappa + weight_dkappa];
        end
        
        function G = CalculateOffset(obj)
            truck_param = GetTruckParams();
            trailer_param = truck_param.trailer;
            length = trailer_param.wheel_base;
            width = 0.0;
            
            kernel_dim = 4 * obj.num_of_knots_ - 1;
            G = zeros(kernel_dim, 1);
            
            for i = 1 : 1 : obj.num_of_knots_
                road_pt = obj.reference_line_(i);
                road_kappa = road_pt.kappa;
                best_k = obj.CalculateBestLaterOffsetParam(road_kappa);
                coeff = obj.CalculateTrailerEdgeLaterOffsetByIndex(i, -length, width);
                G(3 * (i - 1) + 1 : 3 * i, 1) = obj.FormulateMatrix(best_k, coeff).G;
            end
        end
        
        function constriant = CalculateConstraint(obj)
            
            %% 总约束数目：17N - 14
            kernel_dim = 4 * obj.num_of_knots_ - 1;
            num_of_constriant = 17 * obj.num_of_knots_ - 14;
            
            A = zeros(num_of_constriant, kernel_dim);
            lower = zeros(num_of_constriant, 1);
            upper = zeros(num_of_constriant, 1);
            
            rows = 1;
            %% 运动学约束：3(N-1)
            for i = 1 : 1 : obj.num_of_knots_ - 1
                state_space = obj.CalculateStateSpace(i);
                A(rows : rows + 2, 3 * i -2 : 3 * i) = state_space.A;
                A(rows : rows + 2, 3 * (i + 1) -2 : 3 * (i + 1)) = -eye(3);
                A(rows : rows + 2, 3 * obj.num_of_knots_ + i) = state_space.B;
                lower(rows : rows + 2, 1) = -state_space.D;
                upper(rows : rows + 2, 1) = -state_space.D;
                rows = rows + 3;
            end
            
            %% 起点约束：4
            road_kappa = obj.CalculateRoadKappaByIndex(1);
            A(rows : rows + 2, 1 : 3) = eye(3);
            A(rows + 3, 3 * obj.num_of_knots_ + 1) = 1.0;
            lower(rows : rows + 3, 1) = [0.0; 0.0; 0.0; road_kappa];
            upper(rows : rows + 3, 1) = [0.0; 0.0; 0.0; road_kappa];
            rows = rows + 4;
            
            truck_param = GetTruckParams();
            tractor_param = truck_param.tractor;
            
            %% 终点约束: 3, 根据目前的测试场景先不考虑此约束
            
            %% 轮胎位置横向偏移约束(约束在车道内 && 无碰撞区域): 6 * (N - 1)
            truck_param = GetTruckParams();
            
            tractor_param = truck_param.tractor;
            length_tractor = tractor_param.wheel_base;
            width_tractor = tractor_param.width / 2.0;
            
            trailer_param = truck_param.trailer;
            length_trailer = trailer_param.wheel_base;
            width_trailer = trailer_param.width / 2.0;
            
            for i = 1 : 1 : obj.num_of_knots_ - 1
                %% Tractor 右前轴
                s_front_axle_right = obj.CalculateTractorStationByInIndex(i, length_tractor, -width_tractor);
                coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, length_tractor, -width_tractor);
                
                lane_bound_front_axle_right = obj.CalculateBoundByS(obj.lane_boundary_, s_front_axle_right);
                obstacle_bound_front_axle_right = obj.CalculateBoundByS(obj.obstacle_boundary_, s_front_axle_right);
                right_bound = max(lane_bound_front_axle_right.lower, obstacle_bound_front_axle_right.lower);
                left_bound = min(lane_bound_front_axle_right.upper, obstacle_bound_front_axle_right.upper);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = right_bound - coeff.constant;
                upper(rows, 1) = left_bound - coeff.constant;
                %                 disp(strcat('右前轴 constant =  ', 32, num2str(coeff.constant)));
                %                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
                %                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
                rows = rows + 1;
                
                %% Tractor 左前轴
                s_front_axle_left = obj.CalculateTractorStationByInIndex(i, length_tractor, width_tractor);
                coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, length_tractor, width_tractor);
                
                lane_bound_front_axle_left = obj.CalculateBoundByS(obj.lane_boundary_, s_front_axle_left);
                obstalce_bound_front_axle_left = obj.CalculateBoundByS(obj.obstacle_boundary_, s_front_axle_left);
                right_bound = max(lane_bound_front_axle_left.lower, obstalce_bound_front_axle_left.lower);
                left_bound = min(lane_bound_front_axle_left.upper, obstalce_bound_front_axle_left.upper);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = right_bound - coeff.constant;
                upper(rows, 1) = left_bound - coeff.constant;
                %                 disp(strcat('左前轴 constant =  ', 32, num2str(coeff.constant)));
                %                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
                %                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
                rows = rows + 1;
                
                %% Tractor 右后轴
                s_rear_axle_right = obj.CalculateTractorStationByInIndex(i, 0.0, -width_tractor);
                coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, 0.0, -width_tractor);
                
                lane_bound_rear_axle_right = obj.CalculateBoundByS(obj.lane_boundary_, s_rear_axle_right);
                obstacle_bound_rear_axle_right = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_right);
                right_bound = max(lane_bound_rear_axle_right.lower, obstacle_bound_rear_axle_right.lower);
                left_bound = min(lane_bound_rear_axle_right.upper, obstacle_bound_rear_axle_right.upper);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = right_bound - coeff.constant;
                upper(rows, 1) = left_bound - coeff.constant;
                %                 disp(strcat('constant =  ', 32, num2str(coeff.constant)));
                %                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
                %                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
                rows = rows + 1;
                
                %% Tractor 左后轴
                s_rear_axle_left = obj.CalculateTractorStationByInIndex(i, 0.0, width_tractor);
                coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, 0.0, width_tractor);
                
                lane_bound_rear_axle_left = obj.CalculateBoundByS(obj.lane_boundary_, s_rear_axle_left);
                obstacle_bound_rear_axle_left = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_left);
                right_bound = max(lane_bound_rear_axle_left.lower, obstacle_bound_rear_axle_left.lower);
                left_bound = min(lane_bound_rear_axle_left.upper, obstacle_bound_rear_axle_left.upper);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = right_bound - coeff.constant;
                upper(rows, 1) = left_bound - coeff.constant;
                %                 disp(strcat('constant =  ', 32, num2str(coeff.constant)));
                %                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
                %                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
                rows = rows + 1;
                
                %% Trailer 右后轴
                s_rear_axle_right = obj.CalculateTrailerStationByInIndex(i, -length_trailer, -width_trailer);
                coeff = obj.CalculateTrailerEdgeLaterOffsetByIndex(i, -length_trailer, -width_trailer);
                
                lane_bound_rear_axle_right = obj.CalculateBoundByS(obj.lane_boundary_, s_rear_axle_right);
                obstacle_bound_rear_axle_right = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_right);
                right_bound = max(lane_bound_rear_axle_right.lower, obstacle_bound_rear_axle_right.lower);
                left_bound = min(lane_bound_rear_axle_right.upper, obstacle_bound_rear_axle_right.upper);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = right_bound - coeff.constant;
                upper(rows, 1) = left_bound - coeff.constant;
                %                 disp(strcat('constant =  ', 32, num2str(coeff.constant)));
                %                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
                %                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
                rows = rows + 1;
                
                %% Trailer 左后轴
                s_rear_axle_left = obj.CalculateTrailerStationByInIndex(i, -length_trailer, width_trailer);
                coeff = obj.CalculateTrailerEdgeLaterOffsetByIndex(i, -length_trailer, width_trailer);
                
                lane_bound_rear_axle_left = obj.CalculateBoundByS(obj.lane_boundary_, s_rear_axle_left);
                obstacle_bound_rear_axle_left = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_left);
                right_bound = max(lane_bound_rear_axle_left.lower, obstacle_bound_rear_axle_left.lower);
                left_bound = min(lane_bound_rear_axle_left.upper, obstacle_bound_rear_axle_left.upper);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = right_bound - coeff.constant;
                upper(rows, 1) = left_bound - coeff.constant;
                %                 disp(strcat('constant =  ', 32, num2str(coeff.constant)));
                %                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
                %                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
                rows = rows + 1;
                
                
%                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 foh_tractor = tractor_param.front_oh;
%                 roh_tractor = tractor_param.rear_oh;
%                 roh_trailer = trailer_param.rear_oh;
%                  % Tractor 右前角点
%                 s_front_axle_right = obj.CalculateTractorStationByInIndex(i, length_tractor + foh_tractor, -width_tractor);
%                 coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, length_tractor + foh_tractor, -width_tractor);
%                 
%                 lane_bound_front_axle_right = obj.CalculateBoundByS(obj.lane_boundary_, s_front_axle_right);
%                 obstacle_bound_front_axle_right = obj.CalculateBoundByS(obj.obstacle_boundary_, s_front_axle_right);
%                 right_bound = max(lane_bound_front_axle_right.lower, obstacle_bound_front_axle_right.lower);
%                 left_bound = min(lane_bound_front_axle_right.upper, obstacle_bound_front_axle_right.upper);
%                 
%                 A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
%                 A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
%                 lower(rows, 1) = right_bound - coeff.constant;
%                 upper(rows, 1) = left_bound - coeff.constant;
% %                                 disp(strcat('右前轴 constant =  ', 32, num2str(coeff.constant)));
% %                                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
% %                                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
%                 rows = rows + 1;
%                 
%                 % Tractor 左前角点
%                 s_front_axle_left = obj.CalculateTractorStationByInIndex(i, length_tractor + foh_tractor, width_tractor);
%                 coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, length_tractor + foh_tractor, width_tractor);
%                 
%                 lane_bound_front_axle_left = obj.CalculateBoundByS(obj.lane_boundary_, s_front_axle_left);
%                 obstalce_bound_front_axle_left = obj.CalculateBoundByS(obj.obstacle_boundary_, s_front_axle_left);
%                 right_bound = max(lane_bound_front_axle_left.lower, obstalce_bound_front_axle_left.lower);
%                 left_bound = min(lane_bound_front_axle_left.upper, obstalce_bound_front_axle_left.upper);
%                 
%                 A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
%                 A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
%                 lower(rows, 1) = right_bound - coeff.constant;
%                 upper(rows, 1) = left_bound - coeff.constant;
% %                                 disp(strcat('左前轴 constant =  ', 32, num2str(coeff.constant)));
% %                                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
% %                                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
%                 rows = rows + 1;
%                 
%                 % Tractor 右后角点
%                 s_rear_axle_right = obj.CalculateTractorStationByInIndex(i, -roh_tractor, -width_tractor);
%                 coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, -roh_tractor, -width_tractor);
%                 
%                 lane_bound_rear_axle_right = obj.CalculateBoundByS(obj.lane_boundary_, s_rear_axle_right);
%                 obstacle_bound_rear_axle_right = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_right);
%                 right_bound = max(lane_bound_rear_axle_right.lower, obstacle_bound_rear_axle_right.lower);
%                 left_bound = min(lane_bound_rear_axle_right.upper, obstacle_bound_rear_axle_right.upper);
%                 
%                 A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
%                 A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
%                 lower(rows, 1) = right_bound - coeff.constant;
%                 upper(rows, 1) = left_bound - coeff.constant;
% %                                 disp(strcat('constant =  ', 32, num2str(coeff.constant)));
% %                                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
% %                                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
%                 rows = rows + 1;
%                 
%                 % Tractor 左后角点
%                 s_rear_axle_left = obj.CalculateTractorStationByInIndex(i, -roh_tractor, width_tractor);
%                 coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, -roh_tractor, width_tractor);
%                 
%                 lane_bound_rear_axle_left = obj.CalculateBoundByS(obj.lane_boundary_, s_rear_axle_left);
%                 obstacle_bound_rear_axle_left = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_left);
%                 right_bound = max(lane_bound_rear_axle_left.lower, obstacle_bound_rear_axle_left.lower);
%                 left_bound = min(lane_bound_rear_axle_left.upper, obstacle_bound_rear_axle_left.upper);
%                 
%                 A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
%                 A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
%                 lower(rows, 1) = right_bound - coeff.constant;
%                 upper(rows, 1) = left_bound - coeff.constant;
% %                                 disp(strcat('constant =  ', 32, num2str(coeff.constant)));
% %                                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
% %                                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
%                 rows = rows + 1;
%                 
%                 % Trailer 右后角点
%                 s_rear_axle_right = obj.CalculateTrailerStationByInIndex(i, -length_trailer - roh_trailer, -width_trailer);
%                 coeff = obj.CalculateTrailerEdgeLaterOffsetByIndex(i, -length_trailer - roh_trailer, -width_trailer);
%                 
%                 lane_bound_rear_axle_right = obj.CalculateBoundByS(obj.lane_boundary_, s_rear_axle_right);
%                 obstacle_bound_rear_axle_right = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_right);
%                 right_bound = max(lane_bound_rear_axle_right.lower, obstacle_bound_rear_axle_right.lower);
%                 left_bound = min(lane_bound_rear_axle_right.upper, obstacle_bound_rear_axle_right.upper);
%                 
%                 A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
%                 A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
%                 lower(rows, 1) = right_bound - coeff.constant;
%                 upper(rows, 1) = left_bound - coeff.constant;
% %                                 disp(strcat('constant =  ', 32, num2str(coeff.constant)));
% %                                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
% %                                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
%                 rows = rows + 1;
%                 
%                 % Trailer 左后角点
%                 s_rear_axle_left = obj.CalculateTrailerStationByInIndex(i, -length_trailer - roh_trailer, width_trailer);
%                 coeff = obj.CalculateTrailerEdgeLaterOffsetByIndex(i, -length_trailer - roh_trailer, width_trailer);
%                 
%                 lane_bound_rear_axle_left = obj.CalculateBoundByS(obj.lane_boundary_, s_rear_axle_left);
%                 obstacle_bound_rear_axle_left = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_left);
%                 right_bound = max(lane_bound_rear_axle_left.lower, obstacle_bound_rear_axle_left.lower);
%                 left_bound = min(lane_bound_rear_axle_left.upper, obstacle_bound_rear_axle_left.upper);
%                 
%                 A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
%                 A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
%                 lower(rows, 1) = right_bound - coeff.constant;
%                 upper(rows, 1) = left_bound - coeff.constant;
% %                                 disp(strcat('constant =  ', 32, num2str(coeff.constant)));
% %                                 disp(strcat('lower [offset] =  ', 32, num2str(lower(rows, 1))));
% %                                 disp(strcat('upper [offset] =  ', 32, num2str(upper(rows, 1))));
%                 rows = rows + 1;
%                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
            
            
            % 角点位置横向偏移约束(约束在无碰撞区域): 6 * (N - 1)
            foh_tractor = tractor_param.front_oh;
            roh_tractor = tractor_param.rear_oh;
            roh_trailer = trailer_param.rear_oh;
            
            for i = 1 : 1 : obj.num_of_knots_ - 1
                % Tractor 右前角点
                s_front_axle_right = obj.CalculateTractorStationByInIndex(i, length_tractor + foh_tractor, -width_tractor);
                coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, length_tractor + foh_tractor, -width_tractor);
                bound_front_axle_right = obj.CalculateBoundByS(obj.obstacle_boundary_, s_front_axle_right);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = bound_front_axle_right.lower - coeff.constant;
                upper(rows, 1) = bound_front_axle_right.upper - coeff.constant;
                rows = rows + 1;
                
                % Tractor 左前角点
                s_front_axle_left = obj.CalculateTractorStationByInIndex(i, length_tractor + foh_tractor, width_tractor);
                coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, length_tractor + foh_tractor, width_tractor);
                bound_front_axle_left = obj.CalculateBoundByS(obj.obstacle_boundary_, s_front_axle_left);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = bound_front_axle_left.lower - coeff.constant;
                upper(rows, 1) = bound_front_axle_left.upper - coeff.constant;
                rows = rows + 1;
                
                % Tractor 右后角点
                s_rear_axle_right = obj.CalculateTractorStationByInIndex(i, -roh_tractor, -width_tractor);
                coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, -roh_tractor, -width_tractor);
                bound_rear_axle_right = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_right);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = bound_rear_axle_right.lower - coeff.constant;
                upper(rows, 1) = bound_rear_axle_right.upper - coeff.constant;
                rows = rows + 1;
                
                % Tractor 左后角点
                s_rear_axle_left = obj.CalculateTractorStationByInIndex(i, -roh_tractor, width_tractor);
                coeff = obj.CalculateTractorEdgeLaterOffsetByIndex(i, -roh_tractor, width_tractor);
                bound_rear_axle_left = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_left);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = bound_rear_axle_left.lower - coeff.constant;
                upper(rows, 1) = bound_rear_axle_left.upper - coeff.constant;
                rows = rows + 1;
                
                % Trailer 右后角点
                s_rear_axle_right = obj.CalculateTrailerStationByInIndex(i, -length_trailer - roh_trailer, -width_trailer);
                coeff = obj.CalculateTrailerEdgeLaterOffsetByIndex(i, -length_trailer - roh_trailer, -width_trailer);
                bound_rear_axle_right = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_right);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = bound_rear_axle_right.lower - coeff.constant;
                upper(rows, 1) = bound_rear_axle_right.upper - coeff.constant;
                rows = rows + 1;
                
                % Trailer 左后角点
                s_rear_axle_left = obj.CalculateTrailerStationByInIndex(i, -length_trailer - roh_trailer, width_trailer);
                coeff = obj.CalculateTrailerEdgeLaterOffsetByIndex(i, -length_trailer - roh_trailer, width_trailer);
                bound_rear_axle_left = obj.CalculateBoundByS(obj.obstacle_boundary_, s_rear_axle_left);
                
                A(rows, 3 * i - 2 : 3 * i) = [coeff.error_y, coeff.error_theta, coeff.beta];
                A(rows, 3 * obj.num_of_knots_ + i) = coeff.kappa;
                lower(rows, 1) = bound_rear_axle_left.lower - coeff.constant;
                upper(rows, 1) = bound_rear_axle_left.upper - coeff.constant;
                rows = rows + 1;
            end
            
            %% 曲率约束: N-1
            config = GetPathPlannerConfig();
            max_kappa = config.max_kappa;
            max_dkappa = config.max_dkappa * obj.delta_s_;
            A(rows : rows + obj.num_of_knots_ - 2, 3 * obj.num_of_knots_ + 1: kernel_dim) = eye(obj.num_of_knots_ - 1);
            lower(rows : rows + obj.num_of_knots_ - 2, 1) = -max_kappa * ones(obj.num_of_knots_ - 1, 1);
            upper(rows : rows + obj.num_of_knots_ - 2, 1) = max_kappa * ones(obj.num_of_knots_ - 1, 1);
            rows = rows + obj.num_of_knots_ - 1;
            
            % 曲率变化率约束: N-2
            for i = 1 : 1 : obj.num_of_knots_ - 2
                A(rows, 3 * obj.num_of_knots_ + i : 3 * obj.num_of_knots_ + i + 1) = [-1.0, 1.0];
                lower(rows, 1) = -max_dkappa;
                upper(rows, 1) = max_dkappa;
                rows = rows + 1;
            end
            
            constriant.A = A;
            constriant.lower = lower;
            constriant.upper = upper;
            
        end
        
        function param = CalculateBestLaterOffsetParam(obj, road_kappa)
            truck_param = GetTruckParams();
            tractor_param = truck_param.tractor;
            trailer_param = truck_param.trailer;
            
            myfun = @(x, radius, L1, L2, d, foh_tractor, w) ...
                sqrt(x^2 + d^2 - L2^2) - w / 2.0 + sqrt((x + w / 2.0)^2 + (L1 + foh_tractor)^2) - 2 * radius;
            radius = abs(1.0 / (road_kappa + obj.kEpsilon));
            L1 = tractor_param.wheel_base;
            L2 = trailer_param.wheel_base;
            d = truck_param.tractor_base2hinge;
            foh_tractor = tractor_param.front_oh;
            w = trailer_param.width;
            
            fun = @(x) myfun(x, radius, L1, L2, d, foh_tractor, w);
            radius_tractor = fzero(fun, radius);
            
            param = (radius - sqrt(radius_tractor^2 + d^2 - L2^2)) / (radius_tractor - radius);
            param = max(0.0, param);
        end
        
        function coeff = CalculateTractorEdgeLaterOffsetByIndex(obj, index, length, width)
            road_kappa = obj.CalculateRoadKappaByIndex(index);
            road_radius = 1.0 / (road_kappa + obj.kEpsilon);
            
            if road_kappa >= -obj.kEpsilon
                edge_radius = abs(road_radius) + width;
                K = 1.0;
            else
                edge_radius = abs(road_radius) - width;
                K = -1.0;
            end
            
            linearization_reference_pt = obj.GetLinearizationReferencePoint(index);
            error_y_bar = linearization_reference_pt.error_y;
            error_theta_bar = linearization_reference_pt.error_theta;
            beta_bar = linearization_reference_pt.beta;
            kappa_bar = linearization_reference_pt.kappa;
            
            temp = length * cos(error_theta_bar) - width * sin(error_theta_bar) - road_radius * sin(error_theta_bar);
            
            partial.error_y = 1.0;
            partial.error_theta = road_radius * sin(error_theta_bar) - ...
                K * (temp / sqrt(edge_radius^2 - temp^2)) * ...
                (-length * sin(error_theta_bar) - width * cos(error_theta_bar) - road_radius * cos(error_theta_bar));
            partial.beta = 0.0;
            partial.kappa = 0.0;
            partial.constant = error_y_bar - road_radius * cos(error_theta_bar) + ...
                K * sqrt(edge_radius^2 - temp^2) - ...
                partial.error_y * error_y_bar - ...
                partial.error_theta * error_theta_bar - ...
                partial.beta * beta_bar - ...
                partial.kappa * kappa_bar;
            
            coeff = partial;
        end
        
        function coeff = CalculateTrailerEdgeLaterOffsetByIndex(obj, index, length, width)
            road_kappa = obj.CalculateRoadKappaByIndex(index);
            road_radius = 1.0 / (road_kappa + obj.kEpsilon);
            
            if road_kappa >= -obj.kEpsilon
                edge_radius = abs(road_radius) + width;
                K = 1.0;
            else
                edge_radius = abs(road_radius) - width;
                K = -1.0;
            end
            
            linearization_reference_pt = obj.GetLinearizationReferencePoint(index);
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
            %             partial.error_y = 1.0;
            %             partial.error_theta = edge_radius * sin(error_theta_bar + beta_bar) + ...
            %                 K * temp / sqrt(edge_radius ^ 2 - temp ^ 2) * edge_radius * cos(error_theta_bar + beta_bar);
            %             partial.beta = partial.error_theta;
            %             partial.kappa = 0.0;
            %             partial.constant = error_y_bar - road_radius * cos(error_theta_bar + beta_bar) + ...
            %                 K * sqrt(edge_radius^2 - temp^2) - ...
            %                 partial.error_y * error_y_bar - ...
            %                 partial.error_theta * error_theta_bar - ...
            %                 partial.beta * beta_bar - ...
            %                 partial.kappa * kappa_bar;
            %
            %             coeff = partial;
            
            temp = length * cos(error_theta_bar + beta_bar) - ...
                width * sin(error_theta_bar + beta_bar) - ...
                road_radius * sin(error_theta_bar + beta_bar);
            
            partial.error_y = 1.0;
            partial.error_theta = road_radius * sin(error_theta_bar + beta_bar) - ...
                K * (temp / sqrt(edge_radius^2 - temp^2)) * ...
                (-length * sin(error_theta_bar + beta_bar) - ...
                width * cos(error_theta_bar + beta_bar) - road_radius * cos(error_theta_bar + beta_bar));
            partial.beta = partial.error_theta;
            partial.kappa = 0.0;
            partial.constant = error_y_bar - road_radius * cos(error_theta_bar + beta_bar) + ...
                K * sqrt(edge_radius^2 - temp^2) - ...
                partial.error_y * error_y_bar - ...
                partial.error_theta * error_theta_bar - ...
                partial.beta * beta_bar - ...
                partial.kappa * kappa_bar;
            
            coeff = partial;
        end
        
        function road_kappa = CalculateRoadKappaByIndex(obj, index)
            road_pt = obj.reference_line_(index);
            road_kappa = road_pt.kappa;
        end
        
        function linearization_reference_pt = GetLinearizationReferencePoint(obj, index)
            linearization_reference_pt = obj.linearization_reference_(index);
        end
        
        function matrix = FormulateMatrix(obj, k, coeff)
            config = GetPathPlannerConfig();
            H = zeros(3, 3);
            
            H(1, 1) = (k + coeff.error_y) ^ 2;
            H(1, 2) = (k + coeff.error_y) * coeff.error_theta;
            H(1, 3) = (k + coeff.error_y) * coeff.beta;
            
            H(2, 1) = H(1, 2);
            H(2, 2) = coeff.error_theta ^ 2 + config.weight_theta;
            H(2, 3) = coeff.error_theta * coeff.beta;
            
            H(3, 1) = H(1, 3);
            H(3, 2) = H(2, 3);
            H(3, 3) = coeff.beta ^ 2 + config.weight_beta;
            
            G = zeros(3, 1);
            G(1, 1) = (k + coeff.error_y) * coeff.constant;
            G(2, 1) = coeff.error_theta * coeff.constant;
            G(3, 1) = coeff.beta * coeff.constant;
            
            matrix.H = H;
            matrix.G = G;
        end
        
        function state_space = CalculateStateSpace(obj, index)
            road_kappa = obj.CalculateRoadKappaByIndex(index);
            
            linearization_reference_pt = obj.GetLinearizationReferencePoint(index);
            error_y_bar = linearization_reference_pt.error_y;
            error_theta_bar = linearization_reference_pt.error_theta;
            beta_bar = linearization_reference_pt.beta;
            kappa_bar = linearization_reference_pt.kappa;
            
            a_11 = -road_kappa * tan(error_theta_bar);
            a_12 = (1 - road_kappa * error_y_bar) * (1 + (tan(error_theta_bar))^2);
            a_13 = 0.0;
            b_11 = 0.0;
            d_11 = (1 - road_kappa * error_y_bar) * tan(error_theta_bar) - ...
                a_11 * error_y_bar - a_12 * error_theta_bar - a_13 * beta_bar - b_11 * kappa_bar;
            
            a_21 = -road_kappa * kappa_bar / cos(error_theta_bar);
            a_22 = (1 - road_kappa * error_y_bar) * kappa_bar * sin(error_theta_bar) / ((cos(error_theta_bar))^2);
            a_23 = 0.0;
            b_21 = (1 - road_kappa * error_y_bar) / cos(error_theta_bar);
            d_21 = (1 - road_kappa * error_y_bar) * kappa_bar / cos(error_theta_bar) - road_kappa - ...
                a_21 * error_y_bar - a_22 * error_theta_bar - a_23 * beta_bar - b_21 * kappa_bar;
            
            truck_param = GetTruckParams();
            trailer_param = truck_param.trailer;
            L2 = trailer_param.wheel_base;
            d = truck_param.tractor_base2hinge;
            
            a_31 = -road_kappa / cos(error_theta_bar) * (-sin(beta_bar) / L2 - d / L2 * cos(beta_bar) * kappa_bar - kappa_bar);
            a_32 = (1 - road_kappa * error_y_bar) * (-sin(beta_bar) / L2 - d / L2 * cos(beta_bar) * kappa_bar - kappa_bar) ...
                * sin(error_theta_bar) / ((cos(error_theta_bar))^2);
            a_33 = (1 - road_kappa * error_y_bar) / cos(error_theta_bar) * (-cos(beta_bar) / L2 + d / L2 * kappa_bar * sin(beta_bar));
            b_31 = (1 - road_kappa * error_y_bar) / cos(error_theta_bar) * (-d / L2 * cos(beta_bar) - 1.0);
            d_31 = (1 - road_kappa * error_y_bar) / cos(error_theta_bar) * ...
                (-sin(beta_bar) / L2 - d / L2 * cos(beta_bar) * kappa_bar - kappa_bar) - ...
                a_31 * error_y_bar - a_32 * error_theta_bar - a_33 * beta_bar - b_31 * kappa_bar;
            
            state_space.A = [a_11, a_12, a_13;
                a_21, a_22, a_23;
                a_31, a_32, a_33];
            state_space.B = [b_11; b_21; b_31];
            state_space.D = [d_11; d_21; d_31];
            
            state_space = obj.Discretization(state_space);
        end
        
        function state_space_d = Discretization(obj, state_space)
            I = eye(3);
            state_space_d.A = I + obj.delta_s_ * state_space.A;
            state_space_d.B = obj.delta_s_ * state_space.B;
            state_space_d.D = obj.delta_s_ * state_space.D;
        end
        
        function s_hat = CalculateTractorStationByInIndex(obj, index, length, width)
            ref_pt = obj.reference_line_(index);
            s = ref_pt.s;
            
            linearization_reference_pt = obj.GetLinearizationReferencePoint(index);
            error_theta_bar = linearization_reference_pt.error_theta;
            s_hat = s + length * cos(error_theta_bar) - width * sin(error_theta_bar);
        end
        
        function s_hat = CalculateTrailerStationByInIndex(obj, index, length, width)
            ref_pt = obj.reference_line_(index);
            s = ref_pt.s;
            
            linearization_reference_pt = obj.GetLinearizationReferencePoint(index);
            error_theta_bar = linearization_reference_pt.error_theta;
            beta_bar = linearization_reference_pt.beta;
            
            s_hat = s + length * cos(error_theta_bar + beta_bar) - width * sin(error_theta_bar + beta_bar);
        end
        
        function bound = CalculateBoundByS(obj, boundary, s)
            %             num_of_boundary = length(boundary.s);
            %             index = 1;
            %             for i = 1 : 1 : num_of_boundary
            %                 if boundary.s(i) > s
            %                     index = i;
            %                     break;
            %                 end
            %             end
            %
            %             if index > 1 + obj.kEpsilon
            %                 front_index = index - 1;
            %             else
            %                 front_index = index;
            %             end
            front_index = max(min(floor(s / obj.delta_s_), obj.num_of_knots_), 1);
            back_index = min(max(ceil(s / obj.delta_s_), 1), obj.num_of_knots_);
            bound.lower = max(boundary.lower_bound(front_index), ...
                boundary.lower_bound(back_index));
            bound.upper = min(boundary.upper_bound(front_index), ...
                boundary.upper_bound(back_index));
        end
        
        function bound = CalculateBoundByIndex(obj, boundary, index)
            bound.lower = boundary.lower_bound(index);
            bound.upper = boundary.upper_bound(index);
        end
        
    end
end

