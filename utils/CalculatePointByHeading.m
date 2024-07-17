function point = CalculatePointByHeading(pt, length)
point = pt;
point.x = pt.x - length * cos(pt.theta);
point.y = pt.y - length * sin(pt.theta);
point.theta = pt.theta;
end