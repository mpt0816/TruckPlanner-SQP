function linearization_reference = GenerateInitLinearizationReference(lane)
   pt.error_y = 0.0;
   pt.error_theta = 0.0;
   pt.beta = 0.0;
   pt.kappa = 0.0;
   
   linearization_reference = [];
   num = length(lane.center_line);
   last_theta = lane.center_line(1).theta;
   params = GetTruckParams();
   for i = 1 : 1 : num
       rf_pt = lane.center_line(i);
       pt.kappa = rf_pt.kappa;
       if i == 1
           theta = lane.center_line(1).theta;
           pt.beta = 0.0;
       else 
           theta = last_theta + lane.resolution * sin(rf_pt.theta - last_theta) / params.trailer.wheel_base;
           pt.beta = NormalizeAngle(theta - rf_pt.theta);
       end
       last_theta = theta;
       linearization_reference = [linearization_reference, pt];
   end
end

