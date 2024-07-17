function scenarios = Scenario()
    scenarios = [];
    
    %% scenario 1: 直道，没有障碍物
    scenario.name = 'scenario#1#straight-lane';
    key_points = KeyPointStraightPath();
    lane_generator = GenerateLane();
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
    obstacle_type = 'bus';
%     obstacle_type = 'car';
    num_of_obstacle = 0;
    invade = 1.2;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 2: 直道，有障碍物
    scenario.name = 'scenario#2#straight-lane';
    key_points = KeyPointStraightPath();
    lane_generator = GenerateLane();
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
    obstacle_type = 'bus';
%     obstacle_type = 'car';
    num_of_obstacle = 2;
    invade = 1.2;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 3: Left 环路，没有障碍物
    scenario.name = 'scenario#3#left-circle';
    key_points = KeyPointLeftCirclePath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(8);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 0;
    invade = 2.5;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 4: Left 环路，有障碍物
    scenario.name = 'scenario#4#left-circle';
    key_points = KeyPointLeftCirclePath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(8);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 2;
    invade = 2.5;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 5: Right 环路，没有障碍物
    scenario.name = 'scenario#5#right-circle';
    key_points = KeyPointRightCirclePath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(8);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 0;
    invade = 2.5;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 6: Right 环路，有障碍物
    scenario.name = 'scenario#6#right-circle';
    key_points = KeyPointRightCirclePath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(8);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 2;
    invade = 2.5;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 7: Left U型路，没有障碍物
    scenario.name = 'scenario#7#left-U-turn';
    key_points = KeyPointLeftUTurnPath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(6);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 0;
    invade = 1.2;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 8: Left U型路，有障碍物
    scenario.name = 'scenario#8#left-U-turn';
    key_points = KeyPointLeftUTurnPath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(6);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 2;
    invade = 1.2;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 9: Right U型路，没有障碍物
    scenario.name = 'scenario#9#right-U-turn';
    key_points = KeyPointRightUTurnPath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(6);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 0;
    invade = 1.2;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 10: Right U型路，有障碍物
    scenario.name = 'scenario#10#right-U-turn';
    key_points = KeyPointRightUTurnPath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(6);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 2;
    invade = 1.2;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 11: S型路，没有障碍物
    scenario.name = 'scenario#11#S-turn';
    key_points = KeyPointSTurnPath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(8);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 0;
    invade = 1.2;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
    %% scenario 12: S型路，有障碍物
    scenario.name = 'scenario#12#S-turn';
    key_points = KeyPointSTurnPath();
    lane_generator = GenerateLane();
    lane_generator.SetLaneWidth(8);
    lane_generator.SetShoulderHeight(0.1);
    scenario.lane = lane_generator.GenerateLaneWithPointList(key_points);
    scenario.key_points = key_points;
    
%     obstacle_type = 'bus';
    obstacle_type = 'car';
    num_of_obstacle = 8;
    invade = 3.0;
    obstacle_generator = GenerateObstacle(scenario.lane, obstacle_type, num_of_obstacle, invade);
    scenario.obstacles = obstacle_generator.GetObstacles();
    
    scenarios = [scenarios, scenario];
    
end