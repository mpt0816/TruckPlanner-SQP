function PlotIterPlannedPath(scenario, iter_paths)
num_of_iter = length(iter_paths);
%%
ploted_1 = 1;
ploted_5 = num_of_iter;
ploted_3 = floor((ploted_1 + ploted_5) / 2.0);
ploted_2 = floor((ploted_1 + ploted_3) / 2.0);
ploted_4 = floor((ploted_3 + ploted_5) / 2.0);
ploted_index = [ploted_1, ploted_2, ploted_3, ploted_4, ploted_5];
ploted_index = unique(ploted_index);

subplot(1,2,1)
region = PlotEnvironment(scenario);
legend_name{1, 1} = 'Road Center';

i = 2;
for index = ploted_index
    path = iter_paths(index).path;
    PlotTrajectoryLine(path, false, true, false);
    legend_name{1, i} = strcat('SQP Iter # ', num2str(index));
    i = i + 1;
end

xlim([region.min_x, region.max_x]);
ylim([region.min_y, region.max_y + 70.0]);
%     set(gca, 'XLim', [region.min_x, region.max_x]);
%     set(gca, 'YLim', [region.min_y, region.max_y + 20.0]);
xlabel('x(m)');
ylabel('y(m)');
legend(legend_name);
title('Iter PathPlan Result');
%     axis equal;
%     axis off;

%%
index_list = [];
max_offset_eps_abs = [];
max_theta_eps_abs = [];
max_beta_eps_abs = [];
max_kappa_eps_abs = [];

max_offset_eps_rel = [];
max_theta_eps_rel = [];
max_beta_eps_rel = [];
max_kappa_eps_rel = [];

for index = 1 : 1 : length(iter_paths)
    index_list = [index_list, index];
    iter_path = iter_paths(index);
    
    max_offset_eps_abs = [max_offset_eps_abs, iter_path.max_offset_eps_abs];
    max_theta_eps_abs = [max_theta_eps_abs, iter_path.max_theta_eps_abs];
    max_beta_eps_abs = [max_beta_eps_abs, iter_path.max_beta_eps_abs];
    max_kappa_eps_abs = [max_kappa_eps_abs, iter_path.max_kappa_eps_abs];
    
    max_offset_eps_rel = [max_offset_eps_rel, iter_path.max_offset_eps_rel];
    max_theta_eps_rel = [max_theta_eps_rel, iter_path.max_theta_eps_rel];
    max_beta_eps_rel = [max_beta_eps_rel, iter_path.max_beta_eps_rel];
    max_kappa_eps_rel = [max_kappa_eps_rel, iter_path.max_kappa_eps_rel];
end

%%
subplot(2,2,2)
plot(index_list, max_offset_eps_abs, 'LineWidth', 1.0);
hold on;
plot(index_list, max_theta_eps_abs, 'LineWidth', 1.0);
hold on;
plot(index_list, max_beta_eps_abs, 'LineWidth', 1.0);
hold on;
plot(index_list, max_kappa_eps_abs, 'LineWidth', 1.0);

xlabel('Iter');
ylabel('Error');
legend({'Offset', 'Theta', 'Beta', 'kappa'});
title('Absolute Error');

%%
subplot(2,2,4)
plot(index_list, max_offset_eps_rel, 'LineWidth', 1.0);
hold on;
plot(index_list, max_theta_eps_rel, 'LineWidth', 1.0);
hold on;
plot(index_list, max_beta_eps_rel, 'LineWidth', 1.0);
hold on;
plot(index_list, max_kappa_eps_rel, 'LineWidth', 1.0);

xlabel('Iter');
ylabel('Error');
legend({'Offset', 'Theta', 'Beta', 'kappa'});
title('Relative Error');
end

