function pts = CalculateConersFromRearPoint(pt_rear, params)
wheel_base = params.wheel_base;
front_oh = params.front_oh;
rear_oh = params.rear_oh;
width = 0.5 * params.width;

x = pt_rear.x; y = pt_rear.y; theta = pt_rear.theta;
pt1.x = x + (front_oh + wheel_base) * cos(theta) - width * sin(theta);
pt1.y = y + (front_oh + wheel_base) * sin(theta) + width * cos(theta);

pt2.x = x - rear_oh * cos(theta) - width * sin(theta);
pt2.y = y - rear_oh * sin(theta) + width * cos(theta);

pt3.x = x - rear_oh * cos(theta) + width * sin(theta);
pt3.y = y - rear_oh * sin(theta) - width * cos(theta);

pt4.x = x + (front_oh + wheel_base) * cos(theta) + width * sin(theta);
pt4.y = y + (front_oh + wheel_base) * sin(theta) - width * cos(theta);

pts = [pt1, pt2, pt3, pt4];
end