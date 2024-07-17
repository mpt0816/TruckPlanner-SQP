clc; clear all; close all;

%% 添加路径
current_folder = pwd;
addpath(genpath(current_folder));

%% 生成测试场景
scenarios = Scenario();

for scenario = scenarios
    %% 生成规划边界
    boundary_generator = GenerateBoundary(scenario);
    boundaries = boundary_generator.GetBoundary();
    %     region = PlotEnvironment(scenario);
    PlotBoundaries(boundaries);
    
    %% 生成规划模型线性化的初始参考值
    linearization_reference = GenerateInitLinearizationReference(scenario.lane);
    
    %% 生成规划器并求解
    tic;
    planner = SQPPlanner(scenario.lane, boundaries, linearization_reference);
    is_succeed = planner.SolveSQPProblem();
    toc;
    disp(['运行时间: ',num2str(toc)]);
    
    if is_succeed
        path = planner.GetPlannedPath();
        sl_path = planner.GetPlannedSLPath();
        iter_paths = planner.GetIterPlannedPath();
        disp(strcat(scenario.name, ' PathPlan Succeed!'));
        %% 制图
        MakePathPlanGif(scenario, boundaries, path, sl_path, iter_paths, current_folder);
        
    else
        disp(strcat(scenario.name, ' PathPlan Failed!'));
    end
    disp('<=================================================>');
end