function params = GetTruckParams()
tractor.wheel_base = 5.50;
tractor.front_oh = 1.20;
tractor.rear_oh = 0.81;
tractor.width = 2.55;
tractor.length = tractor.wheel_base + tractor.front_oh + tractor.rear_oh;

trailer.wheel_base = 9.0;
trailer.front_oh = 0.85;
trailer.rear_oh = 3.07;
trailer.width = 2.55;
trailer.length = trailer.wheel_base + trailer.front_oh + trailer.rear_oh;

params.tractor = tractor;
params.trailer = trailer;
params.tractor_base2hinge = 0.32;
params.chassis_height = 0.4;
end