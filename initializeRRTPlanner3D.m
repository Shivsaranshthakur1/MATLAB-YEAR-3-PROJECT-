function planner = initializeRRTPlanner3D(dimensions)
 fprintf('Creating SE3 state space...\n');
 ss = stateSpaceSE3;

 % Set state space bounds [x y z qw qx qy qz]
 fprintf('Setting state space bounds...\n');
 bounds = [-20 dimensions(1)+20; % x bounds with buffer
 -20 dimensions(2)+20; % y bounds with buffer
 -10 dimensions(3)+10; % z bounds with buffer
 -1 1; % qw
 -1 1; % qx
 -1 1; % qy
 -1 1]; % qz
 ss.StateBounds = bounds;

 % Create 3D occupancy map with specified resolution
 fprintf('Creating 3D occupancy map...\n');
 resolution = 1; % 1 cell per meter
 map = occupancyMap3D(resolution);

 % Configure map properties - make it more lenient
 map.OccupiedThreshold = 0.80; % Higher threshold for considering cells occupied
 map.FreeThreshold = 0.20; % Lower threshold for considering cells free

 %Initialize entire space as free
 fprintf('Initializing free space...\n');
 [X, Y, Z] = meshgrid(0:resolution:dimensions(1), ...
 0:resolution:dimensions(2), ...
 0:resolution:dimensions(3));
 points = [X(:) Y(:) Z(:)];

% Set all space as explicitly free
 fprintf('Setting space as free (%d points)...\n', size(points,1));
 setOccupancy(map, points, zeros(size(points,1), 1));
% Create 3D validator with proper configuration
 fprintf('Creating state validator...\n');
 validator = validatorOccupancyMap3D(ss);
 validator.Map = map;
 validator.ValidationDistance = 1.0; % Increased for more lenient validation
 validator.XYZIndices = [1 2 3]; % First three state variables are xyz
% Create RRT planner
 fprintf('Creating RRT planner...\n');
 planner = plannerRRT(ss, validator);
 planner.MaxConnectionDistance = 10.0; % Increased for better coverage
 planner.MaxIterations = 10000;
 planner.GoalBias = 0.1;
 fprintf('Planner initialization complete.\n');
end