function PlotPlannedVariables(xy_path, sl_path)

num_of_pts = length(xy_path);
s = zeros(1, num_of_pts);
offset = zeros(1, num_of_pts);
theta_diff = zeros(1, num_of_pts);
beta = zeros(1, num_of_pts);
kappa = zeros(1, num_of_pts);
dkappa = zeros(1, num_of_pts);

for i = 1 : 1 : num_of_pts
    xy_pt = xy_path(i);
    sl_pt = sl_path(i);
    
    s(1, i) = xy_pt.s;
    offset(1, i) = sl_pt.error_y;
    theta_diff(1, i) = sl_pt.error_theta;
    beta(1, i) = xy_pt.beta;
    kappa(1, i) = xy_pt.kappa;
    dkappa(1, i) = xy_pt.dkappa;
end

subplot(4, 1, 1);
plot(s, offset, 'k-', 'LineWidth', 0.5);
xlabel('s(m)');
ylabel('offset(m)');
legend('Tractor Lateral Offset');

subplot(4, 1, 2);
plot(s, theta_diff, 'k-', 'LineWidth', 0.5);
xlabel('s(m)');
ylabel('theta(rad)');
legend('Tractor Heading Difference to ReferenceLine');

subplot(4, 1, 3);
plot(s, beta, 'k-', 'LineWidth', 0.5);
xlabel('s(m)');
ylabel('theta(rad)');
legend('Trailer Heading - Tractor Heading');

subplot(4, 1, 4);
plot(s, kappa, 'k-', 'LineWidth', 0.5);
xlabel('s(m)');
ylabel('kappa(1/m)');
legend('Control Kappa');

end