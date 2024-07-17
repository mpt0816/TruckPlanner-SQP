function norm = NormalizeAngle(angle)
norm = mod(angle + pi, 2.0 * pi);
if norm < 0.0
    norm = norm + pi;
end
norm = norm - pi;
end