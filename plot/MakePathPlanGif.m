function MakePathPlanGif(scenario, boundaries, planned_path, sl_path, iter_paths, file_path)
    
    gif_name = strcat(scenario.name,  '.gif');
    png_name = strcat(scenario.name,  '.png');
    
    legend_buffer = 0.0;
    if strcmp(scenario.name, 'scenario#11#S-turn') || strcmp(scenario.name, 'scenario#12#S-turn')
        legend_buffer = legend_buffer + 30.0;
    end
    
    step = 1;
    path_length = length(planned_path);
    for frame = 1 : 1 : path_length
        if ~(rem(frame - 1, 4) == 0)
            continue;
        end
        close all;
        path = planned_path(1 : frame);
        
        region = PlotEnvironment(scenario);
        PlotPlannedPathArea(path, 1.0, 0.0);
        PlotTruckArea(path(end), true, 1.0, 1.0);
%         PlotTrajectoryLine(path, true, true, true);

        set(gca, 'XLim', [region.min_x, region.max_x]);
        set(gca, 'YLim', [region.min_y, region.max_y + legend_buffer]);
        xlabel('x(m)');
        ylabel('y(m)');
        legend({'Tractor Body', 'Trailer Body'}, ...
            'Location','northeast', 'FontSize', 8);
        axis equal; 
        grid off;
%         axis off;
%         set(gca,'xcolor',[1 1 1]);
%         set(gca,'ycolor',[1 1 1]);
%         set(gca, 'color', [0 0 0]);
%         set(gcf, 'color', [0 0 0]);

%         prettyAxes().dark();
        %% make gif
        pause(0.05);
        MakeGif([file_path, '\figure\', gif_name], step);
        step = step + 1;
    end
    saveas(gcf, [file_path, '\figure\', png_name]);
    close all;

        
    region = PlotEnvironment(scenario);
    PlotPlannedPathArea(planned_path, 1.0, 0.0);
    PlotTruckArea(planned_path(end), true, 0.6, 1.0);
    PlotTrajectoryLine(planned_path, true, true, true);

    set(gca, 'XLim', [region.min_x, region.max_x]);
    set(gca, 'YLim', [region.min_y, region.max_y + legend_buffer]);
    xlabel('x(m)');
    ylabel('y(m)');
    legend({'Road Center', 'Tractor Body', 'Trailer Body', 'Tractor Path', 'Trailer Path'}, ...
        'Location','northeast', 'FontSize', 8);
    axis equal; 

    saveas(gcf, [file_path, '\figure\', png_name]);
    close all;
    
    PlotIterPlannedPath(scenario, iter_paths);
    saveas(gcf, [file_path, '\figure\', strcat(scenario.name,  '-IterPathPlan.png')]);
    close all;
    
    PlotPlannedVariables(planned_path, sl_path);
    saveas(gcf, [file_path, '\figure\', strcat(scenario.name,  '-Variables.png')]);
    close all;
    
end