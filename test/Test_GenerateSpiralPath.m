clc; clear all; close all;

%% 生成测试场景
scenarios = Scenario();

for scenario = scenarios
    spiral_path_generator = GenerateSpiralPath();
    path = spiral_path_generator.GeneratePathWithPointList(scenario.key_points);
    figure;
    PlotSpiralPath(path);
end
