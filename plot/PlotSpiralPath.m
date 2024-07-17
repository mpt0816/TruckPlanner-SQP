function PlotSpiralPath(path)
x = [];
y = [];
s = [];
theta = [];
kappa = [];
dkappa = [];

for pt = path
    x = [x, pt.x];
    y = [y, pt.y];
    s = [s, pt.s];
    theta = [theta, pt.theta];
    kappa = [kappa, pt.kappa];
    dkappa = [dkappa, pt.dkappa];
end

subplot(4, 1, 1);
plot(x, y);
axis equal;
xlabel('x(m)');
ylabel('y(m)');

subplot(4, 1, 2);
plot(s, theta);
xlabel('s(m)');
ylabel('theta(rad)');

subplot(4, 1, 3);
plot(s, kappa);
xlabel('s(m)');
ylabel('kappa(1/m)');

subplot(4, 1, 4);
plot(s, dkappa);
xlabel('s(m)');
ylabel('dkappa(1/m2)');
end

