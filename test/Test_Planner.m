clc; clear all; close all;

%% 添加路径
current_folder = pwd;
addpath(genpath(current_folder));

%% 生成测试场景
scenarios = Scenario();
scenario = scenarios(2);

%% 生成规划边界
boundary_generator = GenerateBoundary(scenario);
boundaries = boundary_generator.GetBoundary();

%% 生成规划模型线性化的初始参考值
linearization_reference = GenerateInitLinearizationReference(scenario.lane);

%% 生成规划器并求解
planner = Planner(scenario.lane, boundaries, linearization_reference);
out = planner.SolveSQPProblem();

if out
    path = planner.GetPlannedPath();
    iter_path = planner.GetIterPlannedPath();
    hold on;
    PlotIterPlannedPath(scenario, iter_path);
end