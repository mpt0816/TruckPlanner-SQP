clc; clear all; close all;

scenarios = Scenario();

for scenario = scenarios
    figure;
    boundary_generator = GenerateBoundary(scenario);
    boundaries = boundary_generator.GetBoundary();
    PlotBoundaries(boundaries);
end
