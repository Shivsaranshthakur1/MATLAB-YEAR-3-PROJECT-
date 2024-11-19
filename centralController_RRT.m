classdef centralController_RRT < handle
    properties
        environment         % Reference to search and rescue environment
        survivorManager    % Manager for survivors
        numAerialVehicles  % Number of UAVs
        numGroundVehicles  % Number of ground vehicles
        aerialPlatforms    % Array of UAV platform objects
        groundPlatforms    % Array of ground vehicle platform objects
        missionStatus      % Current status of the mission
        lastAssignmentTime = 0  % Track when we last made assignments
        REASSIGNMENT_INTERVAL = 5.0  % How often to check for new assignments (seconds
        vehicleAssignments = containers.Map('KeyType', 'char', 'ValueType', 'double')  % Explicitly specify types
        vehicleStatus      % Current status of each vehicle
        
        % New RRT properties
        rrtPlanner         % RRT planner object
        occMap            % Reference to occupancy map
        vehiclePaths      % Store planned paths for each vehicle
    end
    
    properties (Constant)
        % Configuration constants
        CRUISE_HEIGHT = 30     % Default cruise height for UAVs in meters
        MIN_HEIGHT = 10        % Minimum flying height
        MAX_HEIGHT = 80        % Maximum flying height
        SAFE_DISTANCE = 15     % Minimum distance between vehicles
        OBSTACLE_DISTANCE = 20 % Minimum distance from obstacles
        GROUND_SPEED = 15      % Ground vehicle speed in m/s
        AERIAL_SPEED = 25      % Aerial vehicle speed in m/s
        UPDATE_RATE = 50       % Simulation update rate (Hz)
        SIMULATION_STEP = 0.02 % Time step for simulation (seconds)
        
        % RRT Constants
        MAX_PLANNING_ATTEMPTS = 10  % Maximum attempts to find valid path
        REPLAN_DISTANCE = 5.0      % Distance threshold for replanning
    end
    
    methods
        function obj = centralController_RRT(environment, numAerial, numGround)
            % Initialize controller
            obj.environment = environment;
            obj.numAerialVehicles = numAerial;
            obj.numGroundVehicles = numGround;
            obj.missionStatus = 'INITIALIZING';
            
            % Get occupancy map from environment
            obj.occMap = environment.getOccupancyMap();
            
            % Initialize RRT planner
            obj.initializeRRTPlanner();
            
            % Initialize vehicle arrays
            obj.aerialPlatforms = cell(1, numAerial);
            obj.groundPlatforms = cell(1, numGround);
            obj.vehiclePaths = cell(1, numAerial + numGround);

             % Initialize vehicle assignments map with explicit types (ADD THIS LINE)
            obj.vehicleAssignments = containers.Map('KeyType', 'char', 'ValueType', 'double');
            
            
            % Initialize survivor manager and generate survivors
            obj.survivorManager = SurvivorManager(environment);
            numSurvivors = randi([5, 10]);
            obj.survivorManager.generateSurvivors(numSurvivors);
            % Add debug output
            obj.survivorManager.showSurvivorStatus();
            
            % Initialize vehicles
            obj.initializeAerialVehicles();
            obj.initializeGroundVehicles();
        end
        
        function initializeRRTPlanner(obj)
            fprintf('Creating SE3 state space...\n');
            ss = stateSpaceSE3;
            
            % Set state space bounds with buffer
            fprintf('Setting state space bounds...\n');
            bounds = [-20 obj.environment.dimensions(1)+20;    % x bounds with buffer
                      -20 obj.environment.dimensions(2)+20;    % y bounds with buffer
                      -10 obj.environment.dimensions(3)+10;    % z bounds with buffer
                      -1 1;                                    % qw
                      -1 1;                                    % qx
                      -1 1;                                    % qy
                      -1 1];                                   % qz
            ss.StateBounds = bounds;
            
            % Get occupancy map from environment
            obj.occMap = obj.environment.getOccupancyMap();
            
            % Configure map properties for better performance
            obj.occMap.OccupiedThreshold = 0.80;  % Higher threshold for considering cells occupied
            obj.occMap.FreeThreshold = 0.20;      % Lower threshold for considering cells free
            
            % Create state validator with proper configuration
            fprintf('Creating state validator...\n');
            sv = validatorOccupancyMap3D(ss);
            sv.Map = obj.occMap;
            sv.ValidationDistance = 1.0; % Increased for more lenient validation
            sv.XYZIndices = [1 2 3];    % First three state variables are xyz
            
            % Create RRT planner with optimized parameters
            fprintf('Creating RRT planner...\n');
            obj.rrtPlanner = plannerRRT(ss, sv);
            obj.rrtPlanner.MaxConnectionDistance = 10.0; % Increased for better coverage
            obj.rrtPlanner.MaxIterations = 10000;
            obj.rrtPlanner.GoalBias = 0.1;
            
            fprintf('RRT planner initialized successfully\n');
        end
        
        function [success, path] = planPath(obj, startPos, goalPos, isAerial)
            try
                fprintf('Planning path from [%.1f, %.1f, %.1f] to [%.1f, %.1f, %.1f]\n', ...
                startPos(1), startPos(2), startPos(3), goalPos(1), goalPos(2), goalPos(3));
                
                % Debug start and goal positions
                fprintf('Start position validity check: %d\n', obj.isPositionValid(startPos));
                fprintf('Goal position validity check: %d\n', obj.isPositionValid(goalPos));
                
                % Add ground vehicle constraints
                if ~isAerial
                    % Force ground level for ground vehicles
                    startPos(3) = 0;
                    goalPos(3) = 0;
                    
                    % Add extra validation for ground vehicles
                    margin = 2.0;  % Safety margin from obstacles
                    for obs = obj.environment.obstacles.config'
                        % Check if path intersects with obstacle
                        startDist = norm(startPos(1:2) - obs(1:2));
                        goalDist = norm(goalPos(1:2) - obs(1:2));
                        if startDist < (obs(3) + margin) || goalDist < (obs(3) + margin)
                            fprintf('Ground path too close to obstacle\n');
                            success = false;
                            path = [];
                            return;
                        end
                    end
                end
                
                % Verify positions are within environment bounds
                if ~obj.isPositionValid(startPos) || ~obj.isPositionValid(goalPos)
                    fprintf('Start or goal position outside environment bounds\n');
                    success = false;
                    path = [];
                    return;
                end
        
                % Handle different vehicle types
                if ~isAerial && abs(startPos(3)) < 0.1  % Ground vehicle
                    goalPos(3) = 0;  % Force ground level for ground vehicles
                elseif isAerial && abs(goalPos(3)) < 1  % Aerial vehicle approaching ground target
                    % Plan multi-segment path for aerial vehicles approaching ground targets
                    
                    % 1. Plan path to position above survivor
                    abovePos = goalPos;
                    abovePos(3) = obj.CRUISE_HEIGHT;
                    [success1, path1] = planPathSegment(obj, startPos, abovePos);
                    
                    if ~success1
                        fprintf('Failed to plan path to position above target\n');
                        success = false;
                        path = [];
                        return;
                    end
                    
                    % 2. Plan descent to detection height
                    descendPos = goalPos;
                    descendPos(3) = 5;  % Detection height
                    [success2, path2] = planPathSegment(obj, abovePos, descendPos);
                    
                    if ~success2
                        fprintf('Failed to plan descent path\n');
                        success = false;
                        path = [];
                        return;
                    end
                    
                    % 3. Plan return to cruise height
                    returnPos = descendPos;
                    returnPos(3) = obj.CRUISE_HEIGHT;
                    [success3, path3] = planPathSegment(obj, descendPos, returnPos);
                    
                    if ~success3
                        fprintf('Failed to plan return path\n');
                        success = false;
                        path = [];
                        return;
                    end
                    
                    % Combine all path segments
                    path = [path1; path2; path3];
                    success = true;
                    fprintf('Multi-segment aerial path planned successfully\n');
                    fprintf('- Approach waypoints: %d\n', size(path1,1));
                    fprintf('- Descent waypoints: %d\n', size(path2,1));
                    fprintf('- Return waypoints: %d\n', size(path3,1));
                    return;
                end
                
                % Regular single-segment path planning
                % Convert positions to SE3 state vectors [x y z qw qx qy qz]
                startState = zeros(1, 7);
                startState(1:3) = startPos;
                startState(4:7) = [1 0 0 0];  % Default quaternion orientation
                
                goalState = zeros(1, 7);
                goalState(1:3) = goalPos;
                goalState(4:7) = [1 0 0 0];   % Default quaternion orientation
                
                % Debug state vectors
                fprintf('Start state: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]\n', startState);
                fprintf('Goal state: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]\n', goalState);
                
                % Verify states are valid
                if ~isStateValid(obj.rrtPlanner.StateValidator, startState) || ...
                   ~isStateValid(obj.rrtPlanner.StateValidator, goalState)
                    fprintf('Start or goal state in collision with obstacles\n');
                    success = false;
                    path = [];
                    return;
                end
                
                % Plan path using RRT
                fprintf('Computing RRT path...\n');
                [pathObj, solnInfo] = plan(obj.rrtPlanner, startState, goalState);
                
                % Extract and validate path
                path = pathObj.States;
                
                % Verify path validity
                isValid = true;
                for i = 1:size(path,1)-1
                    [motionValid, ~] = isMotionValid(obj.rrtPlanner.StateValidator, ...
                        path(i,:), path(i+1,:));
                    if ~motionValid
                        isValid = false;
                        fprintf('Invalid motion segment at index %d\n', i);
                        break;
                    end
                end
                
                if ~isValid
                    fprintf('Generated path contains invalid segments\n');
                    success = false;
                    path = [];
                    return;
                end
                
                % Path statistics
                fprintf('Path planning successful:\n');
                fprintf('- Path length: %.2f meters\n', pathLength(pathObj));
                fprintf('- Waypoints: %d\n', size(path,1));
                
                success = true;
                
            catch e
                fprintf('Path planning error: %s\n', e.message);
                fprintf('Error details: %s\n', getReport(e));
                success = false;
                path = [];
            end
        end
        
        % Helper function for planning individual path segments
        function [success, path] = planPathSegment(obj, startPos, goalPos)
            % Convert positions to SE3 state vectors
            startState = zeros(1, 7);
            startState(1:3) = startPos;
            startState(4:7) = [1 0 0 0];
            
            goalState = zeros(1, 7);
            goalState(1:3) = goalPos;
            goalState(4:7) = [1 0 0 0];
            
            try
                [pathObj, ~] = plan(obj.rrtPlanner, startState, goalState);
                path = pathObj.States;
                success = true;
            catch
                success = false;
                path = [];
            end
        end

        function valid = isPositionValid(obj, position)
            % Check if position is within environment bounds with safety margin
            margin = 2.0;  % 2-meter safety margin
            valid = position(1) >= margin && position(1) <= (obj.environment.dimensions(1) - margin) && ...
                    position(2) >= margin && position(2) <= (obj.environment.dimensions(2) - margin) && ...
                    position(3) >= 0 && position(3) <= (obj.environment.dimensions(3));
        end
        
        function initializeAerialVehicles(obj)
            fprintf('Initializing %d aerial vehicles...\n', obj.numAerialVehicles);
            % Calculate initial positions in a grid pattern
            spacing = 30;
            gridSize = ceil(sqrt(obj.numAerialVehicles));
            
            for i = 1:obj.numAerialVehicles
                % Calculate grid position
                row = ceil(i / gridSize);
                col = mod(i-1, gridSize) + 1;
                
                % Set initial position
                position = [
                    (col-1) * spacing + spacing/2,...
                    (row-1) * spacing + spacing/2,...
                    obj.CRUISE_HEIGHT
                ];
                
                % Create aerial vehicle using factory
                name = sprintf('UAV%d', i);
                platform = VehicleFactory.createAerialVehicle(...
                    obj.environment.scenario, ...
                    name, ...
                    position, ...
                    [1 0 0], ...        % Red color
                    obj.AERIAL_SPEED);
                fprintf('Created vehicle %s at position [%.1f, %.1f, %.1f]\n', ...
                    name, position(1), position(2), position(3));
                % Store platform
                obj.aerialPlatforms{i} = platform;
                
                % Plan initial path with bounds checking
                margin = 20;  % Safety margin from boundaries
                goalPos = [
                    margin + rand()*(obj.environment.dimensions(1) - 2*margin), ...
                    margin + rand()*(obj.environment.dimensions(2) - 2*margin), ...
                    obj.CRUISE_HEIGHT
                ];
                
                [success, path] = obj.planPath(position, goalPos, true);
                if success
                    obj.vehiclePaths{i} = path;
                else
                    % Try an alternate goal position if first attempt fails
                    goalPos = [
                        position(1) + 50,  % Move 50m in x direction
                        position(2) + 50,  % Move 50m in y direction
                        obj.CRUISE_HEIGHT
                    ];
                    [success, path] = obj.planPath(position, goalPos, true);  % true for aerial
                    if success
                        obj.vehiclePaths{i} = path;
                    end
                end
            end
        end
                
        function initializeGroundVehicles(obj)
            fprintf('Initializing %d ground vehicles...\n', obj.numGroundVehicles);
            
            % Initialize ground vehicles with better boundary margins
            margin = 20; % Increased from 10
            spacing = (obj.environment.dimensions(1) - 2*margin) / (obj.numGroundVehicles + 1);
            
            for i = 1:obj.numGroundVehicles
                % Set initial position along bottom edge with better margins
                position = [
                    margin + i * spacing,... % x position
                    margin,...              % y position with margin
                    0                       % Ground level
                ];
                
                % Create ground vehicle using factory
                name = sprintf('Ground%d', i);
                platform = VehicleFactory.createGroundVehicle(...
                    obj.environment.scenario, ...
                    name, ...
                    position, ...
                    [0 0 1], ...        % Blue color
                    obj.GROUND_SPEED);
                fprintf('Created vehicle %s at position [%.1f, %.1f, %.1f]\n', ...
                    name, position(1), position(2), position(3));
                
                % Store platform
                obj.groundPlatforms{i} = platform;
                
                % Plan initial path with better goal selection
                attempts = 0;
                maxAttempts = 5;
                while attempts < maxAttempts
                    goalPos = [
                        margin + rand()*(obj.environment.dimensions(1) - 2*margin), ...
                        margin + rand()*(obj.environment.dimensions(2) - 2*margin), ...
                        0  % Ground level
                    ];
                    
                    % Check if goal position is valid before planning
                    if obj.isPositionValid(goalPos)
                        [success, path] = obj.planPath(position, goalPos, false);  % false for ground
                        if success
                            obj.vehiclePaths{obj.numAerialVehicles + i} = path;
                            break;
                        end
                    end
                    attempts = attempts + 1;
                end
                
                if attempts == maxAttempts
                    fprintf('Warning: Could not find valid initial path for ground vehicle %d\n', i);
                end
            end
        end
        
        function status = startMission(obj)
            try
                fprintf('\nDebug: Pre-setup state validation:\n');
                % Debug vehicle positions before setup
                for i = 1:obj.numAerialVehicles
                    if ~isempty(obj.aerialPlatforms{i})
                        currentMotion = obj.aerialPlatforms{i}.read();
                        fprintf('Debug: Pre-setup UAV%d position: [%.2f, %.2f, %.2f]\n', ...
                            i, currentMotion(1), currentMotion(2), currentMotion(3));
                    end
                end
        
                for i = 1:obj.numGroundVehicles
                    if ~isempty(obj.groundPlatforms{i})
                        currentMotion = obj.groundPlatforms{i}.read();
                        fprintf('Debug: Pre-setup Ground%d position: [%.2f, %.2f, %.2f]\n', ...
                            i, currentMotion(1), currentMotion(2), currentMotion(3));
                    end
                end
                
                % Debug assignments before setup
                fprintf('Debug: Pre-setup assignments:\n');
                if ~isempty(obj.vehicleAssignments) && obj.vehicleAssignments.Count > 0
                    keys = obj.vehicleAssignments.keys;
                    for k = 1:length(keys)
                        fprintf('Debug: %s assigned to survivor %d\n', ...
                            keys{k}, obj.vehicleAssignments(keys{k}));
                    end
                end
                
                % Store assignments before setup
                preSetupAssignments = containers.Map('KeyType', 'char', 'ValueType', 'double');
                if ~isempty(obj.vehicleAssignments) && obj.vehicleAssignments.Count > 0
                    keys = obj.vehicleAssignments.keys;
                    for k = 1:length(keys)
                        key = char(keys{k});
                        value = double(obj.vehicleAssignments(key));
                        preSetupAssignments(key) = value;
                    end
                end
        
                % Setup scenario
                setup(obj.environment.scenario);
                obj.missionStatus = 'INITIALIZING';
        
                fprintf('\nDebug: Post-setup state validation:\n');
                % Validate and potentially restore vehicle positions
                for i = 1:obj.numAerialVehicles
                    if ~isempty(obj.aerialPlatforms{i})
                        currentMotion = obj.aerialPlatforms{i}.read();
                        fprintf('Debug: Post-setup UAV%d position initial read: [%.2f, %.2f, %.2f]\n', ...
                            i, currentMotion(1), currentMotion(2), currentMotion(3));
                        
                        if isempty(currentMotion) || all(currentMotion(1:3) == 0)
                            fprintf('Debug: Restoring UAV%d position\n', i);
                            initialPos = [15.0, 15.0, obj.CRUISE_HEIGHT];
                            % Create full motion vector with 16 elements
                            newMotion = zeros(1,16);
                            % [1-3]: Position
                            newMotion(1:3) = initialPos;  
                            % [4-6]: Linear velocity
                            newMotion(4:6) = [0 0 0];     
                            % [7-9]: Angular velocity
                            newMotion(7:9) = [0 0 0];  
                            % [10-13]: Orientation quaternion (must be normalized)
                            newMotion(10:13) = [1 0 0 0]; 
                            % [14-16]: Linear acceleration
                            newMotion(14:16) = [0 0 0];   
                            move(obj.aerialPlatforms{i}, newMotion);
                            
                            % Verify restoration
                            currentMotion = obj.aerialPlatforms{i}.read();
                            fprintf('Debug: UAV%d position after restore: [%.2f, %.2f, %.2f]\n', ...
                                i, currentMotion(1), currentMotion(2), currentMotion(3));
                        end
                    end
                end
        
                for i = 1:obj.numGroundVehicles
                    if ~isempty(obj.groundPlatforms{i})
                        currentMotion = obj.groundPlatforms{i}.read();
                        fprintf('Debug: Post-setup Ground%d position initial read: [%.2f, %.2f, %.2f]\n', ...
                            i, currentMotion(1), currentMotion(2), currentMotion(3));
                        
                        if isempty(currentMotion) || all(currentMotion(1:3) == 0)
                            fprintf('Debug: Restoring Ground%d position\n', i);
                            initialPos = [150.0, 20.0, 0];  % Default initial ground position
                            % Create full motion vector with 16 elements
                            newMotion = zeros(1,16);
                            % [1-3]: Position
                            newMotion(1:3) = initialPos;  
                            % [4-6]: Linear velocity
                            newMotion(4:6) = [0 0 0];     
                            % [7-9]: Angular velocity
                            newMotion(7:9) = [0 0 0];  
                            % [10-13]: Orientation quaternion (must be normalized)
                            newMotion(10:13) = [1 0 0 0]; 
                            % [14-16]: Linear acceleration
                            newMotion(14:16) = [0 0 0];   
                            move(obj.groundPlatforms{i}, newMotion);
                            
                            % Verify restoration
                            currentMotion = obj.groundPlatforms{i}.read();
                            fprintf('Debug: Ground%d position after restore: [%.2f, %.2f, %.2f]\n', ...
                                i, currentMotion(1), currentMotion(2), currentMotion(3));
                        end
                    end
                end
                
                % Restore assignments if they were lost
                if obj.vehicleAssignments.Count < preSetupAssignments.Count
                    fprintf('Debug: Assignments were lost during setup. Restoring...\n');
                    obj.vehicleAssignments = containers.Map('KeyType', 'char', 'ValueType', 'double');
                    keys = preSetupAssignments.keys;
                    for k = 1:length(keys)
                        key = char(keys{k});
                        value = double(preSetupAssignments(key));
                        obj.vehicleAssignments(key) = value;
                        fprintf('Debug: Restored assignment %s -> %d\n', key, value);
                    end
                end
                        
                % Create visualization figure
                fig = figure('Name', 'Search and Rescue Mission', 'Position', [100, 100, 1200, 800]);
                
                % Initial scene setup
                ax1 = subplot(1,2,1); % 3D view
                obj.environment.show();
                title('3D View - Initial State');
                view(45, 30);
                lighting gouraud;
                light('Position',[1 1 1]);
                grid on;
                axis equal;
                
                ax2 = subplot(1,2,2); % Top-down view
                obj.environment.show();
                title('Top-Down View - Initial State');
                view(0, 90);
                grid on;
                axis equal;
                
                % Create axes list for easy reference
                axes_list = [ax1, ax2];
                hold(ax1, 'on');
                hold(ax2, 'on');
        
                % Show initial vehicle positions
                for ax = axes_list
                    % Show aerial vehicle starting position
                    for i = 1:obj.numAerialVehicles
                        if ~isempty(obj.aerialPlatforms{i})
                            currentMotion = obj.aerialPlatforms{i}.read();
                            currentPos = currentMotion(1:3);
                            scatter3(ax, currentPos(1), currentPos(2), currentPos(3), 150, 'rd', ...
                                'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
                            text(ax, currentPos(1), currentPos(2), currentPos(3)+5, ...
                                sprintf('UAV%d Start', i), 'Color', 'red');
                        end
                    end
                    % Show ground vehicle starting position
                    for i = 1:obj.numGroundVehicles
                        if ~isempty(obj.groundPlatforms{i})
                            currentMotion = obj.groundPlatforms{i}.read();
                            currentPos = currentMotion(1:3);
                            scatter3(ax, currentPos(1), currentPos(2), 0, 150, 'bd', ... % Force z=0
                                'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
                            text(ax, currentPos(1), currentPos(2), 5, ...
                                sprintf('Ground%d Start', i), 'Color', 'blue');
                        end
                    end
                end
                
                % Show survivors with clear priorities
                for ax = axes_list
                    survivors = obj.survivorManager.Survivors;
                    for i = 1:length(survivors)
                        pos = survivors(i).Position;
                        switch survivors(i).Priority
                            case 1 % High priority
                                h = plot3(ax, pos(1), pos(2), pos(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
                                text(ax, pos(1), pos(2), pos(3)+5, 'High Priority', 'Color', 'red');
                            case 2 % Medium priority
                                h = plot3(ax, pos(1), pos(2), pos(3), 'y*', 'MarkerSize', 15, 'LineWidth', 2);
                                text(ax, pos(1), pos(2), pos(3)+5, 'Medium Priority', 'Color', 'yellow');
                            case 3 % Low priority
                                h = plot3(ax, pos(1), pos(2), pos(3), 'g*', 'MarkerSize', 15, 'LineWidth', 2);
                                text(ax, pos(1), pos(2), pos(3)+5, 'Low Priority', 'Color', 'green');
                        end
                        h.Tag = 'survivor';
                    end
                end
                
                % Pause to show initial state
                fprintf('\nInitial state displayed. Press any key to start path planning...\n');
                pause;
                
                % Start mission execution
                obj.missionStatus = 'PLANNING';
                
                % Assign initial targets (high priority first)
                [assignmentsMade, assignedSurvivorIDs] = obj.assignSurvivorsToVehicles();
                fprintf('\nInitial assignments made: %d survivors assigned to vehicles\n', assignmentsMade);
                
                % Show planned paths
                for ax = axes_list
                    % Plot paths with different colors and styles
                    for i = 1:obj.numAerialVehicles
                        if ~isempty(obj.vehiclePaths{i})
                            path = obj.vehiclePaths{i};
                            plot3(ax, path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2);
                        end
                    end
                    for i = 1:obj.numGroundVehicles
                        idx = obj.numAerialVehicles + i;
                        if ~isempty(obj.vehiclePaths{idx})
                            path = obj.vehiclePaths{idx};
                            plot3(ax, path(:,1), path(:,2), path(:,3), 'b-', 'LineWidth', 2);
                        end
                    end
                end
                
                % Pause to show planned paths
                fprintf('\nPaths planned. Press any key to start movement...\n');
                pause;
                
                % Start movement
                obj.missionStatus = 'ACTIVE';
                lastCheckTime = 0;
                
                while ishandle(fig)
                    currentTime = obj.environment.scenario.CurrentTime;
                    
                    if currentTime - lastCheckTime >= obj.SIMULATION_STEP
                        % Check for collisions
                        obj.checkCollisions();
                        
                        % Check if we should reassign survivors
                        if currentTime - obj.lastAssignmentTime >= obj.REASSIGNMENT_INTERVAL
                            [newAssignments, assignedIDs] = obj.assignSurvivorsToVehicles();
                            if newAssignments > 0
                                fprintf('New assignments made at time %.1f: %d survivors\n', ...
                                    currentTime, newAssignments);
                                obj.lastAssignmentTime = currentTime;
                            end
                        end
                        
                        % Check for survivor detection
                        obj.checkSurvivorDetection();
                        
                        % Update vehicle paths
                        obj.updateVehiclePaths();
                        
                        % Clear and update visualization
                        for ax = axes_list
                            children = get(ax, 'Children');
                            for child = children'
                                if ~strcmp(get(child, 'Tag'), 'survivor') && ...
                                   ~strcmp(get(child, 'Tag'), 'environment')
                                    delete(child);
                                end
                            end
                            hold(ax, 'on');
                            
                            % Update vehicle positions and paths
                            obj.visualizeCurrentState(ax);
                        end
                        
                        lastCheckTime = currentTime;
                        drawnow;
                    end
                    
                    if ~advance(obj.environment.scenario)
                        break;
                    end
                    
                    pause(obj.SIMULATION_STEP/2);
                end
                
                status = true;
                
            catch e
                fprintf('\nError in mission: %s\n', e.message);
                fprintf('Error stack trace:\n%s\n', getReport(e));
                status = false;
            end
        end
       function clearPlots(ax)
            % Clear all plot objects but keep scatter points (survivors)
            children = get(ax, 'Children');
            for child = children'
                if ~isa(child, 'matlab.graphics.chart.primitive.Scatter')
                    delete(child);
                end
            end
            hold(ax, 'on');
       end

       % Add survivor detection checking
     function checkSurvivorDetection(obj)
            DETECTION_RADIUS = 5.0;  % meters
            AERIAL_DETECTION_HEIGHT = 10.0; % meters - new parameter for descent
            
            % Get all keys and ensure they're char type
            if ~isempty(obj.vehicleAssignments) && obj.vehicleAssignments.Count > 0
                assignmentKeys = obj.vehicleAssignments.keys;
                
                for k = 1:length(assignmentKeys)
                    try
                        % Ensure vehicleID is char type when accessing map
                        vehicleID = char(assignmentKeys{k});
                        survivorID = obj.vehicleAssignments(vehicleID);
                        
                        % Get survivor object
                        survivor = obj.findSurvivorByID(survivorID);
                        
                        if ~isempty(survivor) && strcmp(survivor.Status, 'IN_PROGRESS')
                            % Different handling for aerial and ground vehicles
                            if contains(vehicleID, 'UAV')
                                idx = str2double(vehicleID(4:end));
                                platform = obj.aerialPlatforms{idx};
                                currentMotion = platform.read();
                                vehiclePos = currentMotion(1:3);
                                
                                % Get horizontal and vertical distances separately
                                horizontal_dist = norm(survivor.Position(1:2) - vehiclePos(1:2));
                                height_diff = abs(vehiclePos(3) - (survivor.Position(3) + AERIAL_DETECTION_HEIGHT));
                                
                                fprintf('Debug: UAV%d - Horizontal dist: %.2f, Height diff: %.2f\n', ...
                                    idx, horizontal_dist, height_diff);
                                
                                % If we're close horizontally, initiate descent
                                if horizontal_dist < DETECTION_RADIUS * 2
                                    descentPos = vehiclePos;
                                    descentPos(3) = survivor.Position(3) + AERIAL_DETECTION_HEIGHT;
                                    [success, path] = obj.planPath(vehiclePos, descentPos, true);
                                    if success
                                        fprintf('Debug: Planning descent for UAV%d\n', idx);
                                        obj.vehiclePaths{idx} = path;
                                    end
                                end
                                
                                % Check for detection only when both horizontal and vertical distances are small
                                if horizontal_dist < DETECTION_RADIUS && height_diff < 2.0
                                    fprintf('Vehicle %s detected survivor %d\n', vehicleID, survivorID);
                                    survivor.Status = 'DETECTED';
                                    obj.vehicleAssignments.remove(vehicleID);
                                    
                                    % Plan return to cruise height
                                    returnPos = vehiclePos;
                                    returnPos(3) = obj.CRUISE_HEIGHT;
                                    [success, path] = obj.planPath(vehiclePos, returnPos, true);
                                    if success
                                        fprintf('Debug: Planned return path for %s after detection\n', vehicleID);
                                        obj.vehiclePaths{idx} = path;
                                    end
                                end
                            else
                                % Ground vehicle logic
                                idx = str2double(vehicleID(7:end));
                                platform = obj.groundPlatforms{idx};
                                if ~isempty(platform)
                                    currentMotion = platform.read();
                                    vehiclePos = currentMotion(1:3);
                                    dist = norm(survivor.Position - vehiclePos);
                                    
                                    fprintf('Debug: Ground%d - Distance to survivor: %.2f\n', idx, dist);
                                    
                                    if dist < DETECTION_RADIUS
                                        fprintf('Vehicle %s detected survivor %d\n', vehicleID, survivorID);
                                        survivor.Status = 'DETECTED';
                                        obj.vehicleAssignments.remove(vehicleID);
                                    end
                                    
                                    % Debug output for ground vehicle movement
                                    fprintf('Debug: Ground%d - Current position: [%.2f, %.2f, %.2f], Velocity: [%.2f, %.2f, %.2f]\n', ...
                                        idx, vehiclePos(1), vehiclePos(2), vehiclePos(3), ...
                                        currentMotion(4), currentMotion(5), currentMotion(6));
                                else
                                    fprintf('Debug: Ground%d platform is empty\n', idx);
                                end
                            end
                        end
                    catch e
                        fprintf('Debug: Error checking detection for vehicle %s: %s\n', ...
                            vehicleID, e.message);
                        fprintf('Debug: Error stack trace: %s\n', getReport(e));
                    end
                end
            end
        end

        function survivor = findSurvivorByID(obj, survivorID)
            survivor = [];
            for s = obj.survivorManager.Survivors
                if s.ID == survivorID
                    survivor = s;
                    break;
                end
            end
        end


        function pos = getVehiclePosition(obj, vehicleID)
            if contains(vehicleID, 'UAV')
                idx = str2double(vehicleID(4:end));
                platform = obj.aerialPlatforms{idx};
            else
                idx = str2double(vehicleID(7:end));
                platform = obj.groundPlatforms{idx};
            end
            currentMotion = platform.read();
            pos = currentMotion(1:3);
        end


      function visualizeCurrentState(obj, ax)
            % Plot aerial vehicles and their paths
            for i = 1:obj.numAerialVehicles
                if ~isempty(obj.aerialPlatforms{i})
                    currentMotion = obj.aerialPlatforms{i}.read();
                    currentPos = currentMotion(1:3);
                    
                    % Plot current position
                    scatter3(ax, currentPos(1), currentPos(2), currentPos(3), 100, 'ro', ...
                        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2, ...
                        'Tag', 'vehicle');
                    
                    % Plot only the current active path
                    if ~isempty(obj.vehiclePaths{i})
                        path = obj.vehiclePaths{i};
                        h = plot3(ax, path(:,1), path(:,2), path(:,3), 'r-', ...
                            'LineWidth', 2, 'Tag', 'path');
                        
                        % Add direction arrow
                        if size(path, 1) > 1
                            quiver3(ax, currentPos(1), currentPos(2), currentPos(3), ...
                                path(1,1)-currentPos(1), ...
                                path(1,2)-currentPos(2), ...
                                path(1,3)-currentPos(3), ...
                                0, 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
                        end
                    end
                end
            end
            
            % Plot ground vehicles and their paths
            for i = 1:obj.numGroundVehicles
                if ~isempty(obj.groundPlatforms{i})
                    currentMotion = obj.groundPlatforms{i}.read();
                    currentPos = currentMotion(1:3);
                    
                    % Plot current position
                    scatter3(ax, currentPos(1), currentPos(2), currentPos(3), 100, 'bs', ...
                        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2, ...
                        'Tag', 'vehicle');
                    
                    % Plot only the current active path
                    idx = obj.numAerialVehicles + i;
                    if ~isempty(obj.vehiclePaths{idx})
                        path = obj.vehiclePaths{idx};
                        h = plot3(ax, path(:,1), path(:,2), path(:,3), 'b-', ...
                            'LineWidth', 2, 'Tag', 'path');
                        
                        % Add direction arrow
                        if size(path, 1) > 1
                            quiver3(ax, currentPos(1), currentPos(2), currentPos(3), ...
                                path(1,1)-currentPos(1), ...
                                path(1,2)-currentPos(2), ...
                                0, ... % Keep z component 0 for ground vehicle
                                0, 'b', 'LineWidth', 2, 'MaxHeadSize', 1);
                        end
                    end
                end
            end
            
            % Add vehicle labels
            for i = 1:obj.numAerialVehicles
                if ~isempty(obj.aerialPlatforms{i})
                    currentMotion = obj.aerialPlatforms{i}.read();
                    currentPos = currentMotion(1:3);
                    text(ax, currentPos(1), currentPos(2), currentPos(3)+5, ...
                        sprintf('UAV%d', i), 'Color', 'red');
                end
            end
            
            for i = 1:obj.numGroundVehicles
                if ~isempty(obj.groundPlatforms{i})
                    currentMotion = obj.groundPlatforms{i}.read();
                    currentPos = currentMotion(1:3);
                    text(ax, currentPos(1), currentPos(2), currentPos(3)+5, ...
                        sprintf('Ground%d', i), 'Color', 'blue');
                end
            end
        end
                        
        function updateVehiclePaths(obj)
            % Update paths for all vehicles if needed
            for i = 1:obj.numAerialVehicles
                if ~isempty(obj.aerialPlatforms{i}) && ~isempty(obj.vehiclePaths{i})
                    currentMotion = obj.aerialPlatforms{i}.read();
                    currentPos = currentMotion(1:3);
                    
                    % Debug current state
                    fprintf('Debug: UAV%d updatePaths - Current pos: [%.2f, %.2f, %.2f]\n', ...
                        i, currentPos(1), currentPos(2), currentPos(3));
                    
                    % Check if we need to replan
                    if obj.needsReplanning(currentPos, obj.vehiclePaths{i})
                        fprintf('Debug: Replanning path for aerial vehicle %d\n', i);
                        goalPos = obj.vehiclePaths{i}(end, 1:3);
                        [success, newPath] = obj.planPath(currentPos, goalPos, true);
                        
                        if success
                            obj.vehiclePaths{i} = newPath;
                            fprintf('Debug: Successfully replanned path for UAV%d\n', i);
                        end
                    end
                    
                    % Update vehicle position along path
                    if ~isempty(obj.vehiclePaths{i})
                        obj.vehiclePaths{i} = obj.followPathSegment(obj.aerialPlatforms{i}, obj.vehiclePaths{i}, true);
                    end
                end
            end
            
            % Same for ground vehicles but ensure z=0
            for i = 1:obj.numGroundVehicles
                idx = obj.numAerialVehicles + i;
                if ~isempty(obj.groundPlatforms{i}) && ~isempty(obj.vehiclePaths{idx})
                    currentMotion = obj.groundPlatforms{i}.read();
                    currentPos = currentMotion(1:3);
                    
                    fprintf('Debug: Ground%d updatePaths - Current pos: [%.2f, %.2f, %.2f]\n', ...
                        i, currentPos(1), currentPos(2), currentPos(3));
                    
                    % Force ground vehicle path to stay at z=0
                    if any(obj.vehiclePaths{idx}(:,3) ~= 0)
                        obj.vehiclePaths{idx}(:,3) = zeros(size(obj.vehiclePaths{idx}, 1), 1);
                        fprintf('Debug: Forced ground path to z=0 for Ground%d\n', i);
                    end
                    
                    % Check if we need to replan
                    if obj.needsReplanning(currentPos, obj.vehiclePaths{idx})
                        fprintf('Debug: Replanning path for ground vehicle %d\n', i);
                        goalPos = obj.vehiclePaths{idx}(end, 1:3);
                        goalPos(3) = 0;  % Force goal to ground level
                        [success, newPath] = obj.planPath(currentPos, goalPos, false);
                        
                        if success
                            obj.vehiclePaths{idx} = newPath;
                            % Force z=0 again just to be safe
                            obj.vehiclePaths{idx}(:,3) = zeros(size(newPath, 1), 1);
                            fprintf('Debug: Successfully replanned path for Ground%d\n', i);
                        end
                    end
                    
                    % Update vehicle position along path
                    if ~isempty(obj.vehiclePaths{idx})
                        obj.vehiclePaths{idx} = obj.followPathSegment(obj.groundPlatforms{i}, obj.vehiclePaths{idx}, false);
                    end
                end
            end
        end
        function path = followPathSegment(obj, platform, path, isAerial)
            if size(path, 1) < 2
                return;
            end
            
            % Get current state
            currentMotion = platform.read();
            currentPos = currentMotion(1:3);
            fprintf('Debug: followPathSegment - Current pos: [%.2f, %.2f, %.2f]\n', ...
                currentPos(1), currentPos(2), currentPos(3));
            
            % Get next waypoint
            nextWaypoint = path(1, 1:3);
            if ~isAerial
                nextWaypoint(3) = 0;  % Force ground vehicles to z=0
            end
            
            % Calculate direction and distance
            direction = nextWaypoint - currentPos;
            distance = norm(direction);
            
            fprintf('Debug: Distance to next waypoint: %.2f\n', distance);
            
            if distance < obj.REPLAN_DISTANCE
                % Remove reached waypoint
                path(1,:) = [];
                fprintf('Debug: Waypoint reached, %d waypoints remaining\n', size(path,1));
                return;
            end
            
            % Normalize direction and scale by vehicle speed
            direction = direction / distance;
            
            if isAerial
                speed = obj.AERIAL_SPEED;
            else
                speed = obj.GROUND_SPEED;
            end
            
            velocity = direction * speed;
            
            % Create motion vector
            newMotion = zeros(1, 16);  % Full motion vector
            newMotion(1:3) = currentPos + velocity * obj.SIMULATION_STEP; % Position
            newMotion(4:6) = velocity; % Velocity
            newMotion(10:13) = [1 0 0 0];  % Orientation quaternion
            
            % For ground vehicles, ensure z=0
            if ~isAerial
                newMotion(3) = 0; % Z position
                newMotion(6) = 0; % Z velocity
            end
            
            % Move platform
            move(platform, newMotion);
            fprintf('Debug: Moving to [%.2f, %.2f, %.2f] with velocity [%.2f, %.2f, %.2f]\n', ...
                newMotion(1), newMotion(2), newMotion(3), ...
                newMotion(4), newMotion(5), newMotion(6));
        end
       function needsReplan = needsReplanning(obj, currentPos, path)
            % Always replan if path is empty
            if isempty(path)
                needsReplan = true;
                return;
            end
            
            % Check distance to next waypoint
            nextWaypoint = path(1, 1:3);
            distanceToNext = norm(currentPos - nextWaypoint);
            
            % Check if near final waypoint
            if size(path, 1) == 1 && distanceToNext < obj.REPLAN_DISTANCE
                needsReplan = true;
                return;
            end
            
            % Check if current path segment is still valid
            if size(path, 1) > 1
                [isValid, ~] = isMotionValid(obj.rrtPlanner.StateValidator, ...
                    [currentPos, 1, 0, 0, 0], ...
                    [nextWaypoint, 1, 0, 0, 0]);
                if ~isValid
                    needsReplan = true;
                    return;
                end
            end
            
            needsReplan = false;
        end
        
        function collisionDetected = checkCollisions(obj)
            try
                collisionDetected = false;
                
                % Create temporary assignments map with same types as original
                tempAssignments = containers.Map('KeyType', 'char', 'ValueType', 'double');
                
                % Debug current assignments state
                fprintf('Debug: Initial vehicle assignments state:\n');
                if ~isempty(obj.vehicleAssignments) && obj.vehicleAssignments.Count > 0
                    keys = obj.vehicleAssignments.keys;
                    for k = 1:length(keys)
                        fprintf('Debug: %s assigned to survivor %d\n', ...
                            keys{k}, obj.vehicleAssignments(keys{k}));
                    end
                else
                    fprintf('Debug: No current assignments\n');
                end
                
                % Safely copy existing assignments with type validation
                if ~isempty(obj.vehicleAssignments) && obj.vehicleAssignments.Count > 0
                    keys = obj.vehicleAssignments.keys;
                    fprintf('Debug: Copying assignments to temporary map...\n');
                    for k = 1:length(keys)
                        key = char(keys{k});
                        value = double(obj.vehicleAssignments(key));
                        fprintf('Debug: Copying assignment %s -> %d (types: %s -> %s)\n', ...
                            key, value, class(key), class(value));
                        tempAssignments(key) = value;
                    end
                end
        
                % Check collisions between aerial and ground vehicles
                for i = 1:obj.numAerialVehicles
                    if ~isempty(obj.aerialPlatforms{i})
                        currentMotion = obj.aerialPlatforms{i}.read();
                        % Debug vehicle positions
                        fprintf('Debug: UAV%d position read: ', i);
                        if isempty(currentMotion)
                            fprintf('empty motion\n');
                            continue;
                        elseif all(currentMotion(1:3) == 0)
                            fprintf('zeroed position [0, 0, 0]\n');
                            continue;
                        else
                            fprintf('[%.2f, %.2f, %.2f]\n', ...
                                currentMotion(1), currentMotion(2), currentMotion(3));
                        end
                        
                        pos1 = currentMotion(1:3);
                        
                        for j = 1:obj.numGroundVehicles
                            if ~isempty(obj.groundPlatforms{j})
                                otherMotion = obj.groundPlatforms{j}.read();
                                % Debug ground vehicle position
                                fprintf('Debug: Ground%d position read: ', j);
                                if isempty(otherMotion)
                                    fprintf('empty motion\n');
                                    continue;
                                elseif all(otherMotion(1:3) == 0)
                                    fprintf('zeroed position [0, 0, 0]\n');
                                    continue;
                                else
                                    fprintf('[%.2f, %.2f, %.2f]\n', ...
                                        otherMotion(1), otherMotion(2), otherMotion(3));
                                end
                                
                                pos2 = otherMotion(1:3);
                                
                                % Check horizontal distance
                                dist = norm(pos1(1:2) - pos2(1:2));
                                fprintf('Debug: Distance between UAV%d and Ground%d: %.2f\n', ...
                                    i, j, dist);
                                
                                if dist < obj.SAFE_DISTANCE
                                    collisionDetected = true;
                                    
                                    % Create vehicle IDs with explicit type checking
                                    aerialID = char(sprintf('UAV%d', i));
                                    groundID = char(sprintf('Ground%d', j));
                                    
                                    fprintf('Debug: Collision detected between %s and %s\n', ...
                                        aerialID, groundID);
                                    fprintf('Debug: Vehicle positions - Aerial: [%.2f, %.2f, %.2f], Ground: [%.2f, %.2f, %.2f]\n', ...
                                        pos1(1), pos1(2), pos1(3), pos2(1), pos2(2), pos2(3));
                                    
                                    % Debug assignments before collision handling
                                    fprintf('Debug: Assignments before collision handling:\n');
                                    if tempAssignments.Count > 0
                                        tempKeys = tempAssignments.keys;
                                        for k = 1:length(tempKeys)
                                            fprintf('Debug: %s -> %d\n', ...
                                                char(tempKeys{k}), double(tempAssignments(tempKeys{k})));
                                        end
                                    end
                                    
                                    % Perform collision avoidance
                                    obj.adjustAerialHeight(i, pos1, currentMotion(4:6));
                                    fprintf('Debug: Adjusted height for UAV%d\n', i);
                                    
                                    % Restore assignments with type validation
                                    if tempAssignments.Count > 0
                                        fprintf('Debug: Restoring assignments after collision handling\n');
                                        tempKeys = tempAssignments.keys;
                                        obj.vehicleAssignments = containers.Map('KeyType', 'char', 'ValueType', 'double');
                                        for k = 1:length(tempKeys)
                                            key = char(tempKeys{k});
                                            val = double(tempAssignments(key));
                                            fprintf('Debug: Restoring %s -> %d\n', key, val);
                                            obj.vehicleAssignments(key) = val;
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
                
            catch e
                fprintf('Error in collision detection: %s\n', e.message);
                fprintf('Stack trace:\n%s\n', getReport(e));
                rethrow(e);  % Rethrow to see complete error chain
            end
        end
                function avoidAerialCollision(obj, i, j, pos1, pos2, vel1, vel2)
            % Calculate relative position and velocity
            relPos = pos2 - pos1;
            relVel = vel2 - vel1;
            
            % Calculate time to closest approach
            timeToClosest = max(0, -dot(relPos, relVel) / (norm(relVel)^2 + 1e-6));
            
            % Calculate avoidance velocities
            platform1 = obj.aerialPlatforms{i};
            platform2 = obj.aerialPlatforms{j};
            
            % Get current states
            motion1 = platform1.read();
            motion2 = platform2.read();
            
            % Adjust heights in opposite directions
            motion1(3) = pos1(3) + 5;  % Move up
            motion2(3) = pos2(3) - 5;  % Move down
            
            % Keep original velocities but adjust vertical component
            motion1(6) = 2.0;  % Upward velocity
            motion2(6) = -2.0; % Downward velocity
            
            % Move platforms
            move(platform1, motion1);
            move(platform2, motion2);
        end

        function adjustAerialHeight(obj, i, pos, vel)
            platform = obj.aerialPlatforms{i};
            currentMotion = platform.read();
            
            % Calculate new height maintaining horizontal position
            newMotion = currentMotion;
            newMotion(3) = min(obj.MAX_HEIGHT, pos(3) + 10);  % Increase height
            newMotion(6) = 2.0;  % Upward velocity
            
            % Move platform
            move(platform, newMotion);
        end

        function avoidGroundCollision(obj, i, j, pos1, pos2, vel1, vel2)
            % Calculate avoidance direction perpendicular to current motion
            platform1 = obj.groundPlatforms{i};
            platform2 = obj.groundPlatforms{j};
            
            % Get current states
            motion1 = platform1.read();
            motion2 = platform2.read();
            
            % Calculate perpendicular directions
            dir = pos1(1:2) - pos2(1:2);
            perp = [-dir(2), dir(1)];
            perp = perp / (norm(perp) + 1e-6) * obj.GROUND_SPEED;
            
            % Update motions maintaining z=0
            motion1(1:2) = pos1(1:2) + perp;
            motion2(1:2) = pos2(1:2) - perp;
            
            % Update velocities
            motion1(4:5) = perp;
            motion2(4:5) = -perp;
            
            % Ensure z=0 for ground vehicles
            motion1(3) = 0;
            motion2(3) = 0;
            motion1(6) = 0;
            motion2(6) = 0;
            
            % Move platforms
            move(platform1, motion1);
            move(platform2, motion2);
        end

        function [assignmentsMade, assignedSurvivorIDs] = assignSurvivorsToVehicles(obj)
            % Initialize outputs
            assignmentsMade = 0;
            assignedSurvivorIDs = [];

            % Initialize vehicle assignments Map with explicit types if empty
            
            if isempty(obj.vehicleAssignments)
                obj.vehicleAssignments = containers.Map('KeyType', 'char', 'ValueType', 'double');
            end
                
            % Get undetected survivors
            undetectedSurvivors = obj.survivorManager.getUndetectedSurvivors();
            fprintf('\nChecking survivor assignments...\n');
            obj.survivorManager.showSurvivorStatus();
        
            if isempty(undetectedSurvivors)
                fprintf('No undetected survivors to assign.\n');
                return;
            end
        
            fprintf('Found %d undetected survivors\n', length(undetectedSurvivors));
        
            % Count high priority survivors
            highPriorityCount = 0;
            for s = 1:length(undetectedSurvivors)
                if undetectedSurvivors(s).Priority == 1
                    highPriorityCount = highPriorityCount + 1;
                end
            end
            fprintf('Found %d undetected survivors (%d high priority)\n', ...
                length(undetectedSurvivors), highPriorityCount);
        
            % First assign aerial vehicles
            for i = 1:obj.numAerialVehicles
                if ~isempty(obj.aerialPlatforms{i})
                    % ADD AT START OF AERIAL VEHICLE LOOP
                    vehicleID = sprintf('UAV%d', i);
                   if obj.vehicleAssignments.isKey(vehicleID)
                        continue;  % Skip if vehicle already has an assignment
                    end
        
                    % Get current vehicle state and ensure it's valid
                    currentMotion = obj.aerialPlatforms{i}.read();
                    if isempty(currentMotion) || all(currentMotion(1:3) == 0)
                        % Use initialization position if current position is invalid
                        spacing = 30;  % Same as in initializeAerialVehicles
                        gridSize = ceil(sqrt(obj.numAerialVehicles));
                        row = ceil(i / gridSize);
                        col = mod(i-1, gridSize) + 1;
                        currentPos = [
                            (col-1) * spacing + spacing/2, ...
                            (row-1) * spacing + spacing/2, ...
                            obj.CRUISE_HEIGHT
                        ];
                        fprintf('Using initial position for UAV%d: [%.1f, %.1f, %.1f]\n', ...
                            i, currentPos(1), currentPos(2), currentPos(3));
                    else
                        currentPos = currentMotion(1:3);
                        fprintf('Current position for UAV%d: [%.1f, %.1f, %.1f]\n', ...
                            i, currentPos(1), currentPos(2), currentPos(3));
                    end
        
                    % First try to assign high priority survivors
                    survivorAssigned = false;
                    for s = 1:length(undetectedSurvivors)
                        if undetectedSurvivors(s).Priority == 1 && ...
                                strcmp(undetectedSurvivors(s).Status, 'UNDETECTED')
                            goalPos = undetectedSurvivors(s).Position;
                            goalPos(3) = obj.CRUISE_HEIGHT; % Maintain cruise height
                            [success, path] = obj.planPath(currentPos, goalPos, true);
                            if success
                                obj.vehiclePaths{i} = path;
                                assignmentsMade = assignmentsMade + 1;
                                assignedSurvivorIDs = [assignedSurvivorIDs, undetectedSurvivors(s).ID];
                                fprintf('Assigned high-priority survivor %d to UAV%d (distance: %.1fm)\n', ...
                                    undetectedSurvivors(s).ID, i, norm(goalPos - currentPos));
                                survivorAssigned = true;
                                break;
                            else
                                fprintf('Failed to plan path to high-priority survivor %d\n', ...
                                    undetectedSurvivors(s).ID);
                            end
                        end
                    end
        
                    % If no high priority survivor was assigned, try other survivors
                    if ~survivorAssigned
                        % Find nearest unassigned survivor
                        minDist = inf;
                        nearestSurvivor = [];
                        nearestIdx = 0;
        
                        for s = 1:length(undetectedSurvivors)
                            if strcmp(undetectedSurvivors(s).Status, 'UNDETECTED')
                                dist = norm(undetectedSurvivors(s).Position - currentPos);
                                if dist < minDist
                                    minDist = dist;
                                    nearestSurvivor = undetectedSurvivors(s);
                                    nearestIdx = s;
                                end
                            end
                        end
        
                        if ~isempty(nearestSurvivor)
                            goalPos = nearestSurvivor.Position;
                            goalPos(3) = obj.CRUISE_HEIGHT; % Maintain cruise height
                            [success, path] = obj.planPath(currentPos, goalPos, true);
        
                            % REPLACEMENT BLOCK FOR AERIAL VEHICLES
                            if success
                            
                            obj.vehiclePaths{i} = path;
                            undetectedSurvivors(nearestIdx).Status = 'IN_PROGRESS';
                            vehicleID = char(sprintf('UAV%d', i));  % Ensure char type
                            if ~obj.vehicleAssignments.isKey(vehicleID)
                                obj.vehicleAssignments(vehicleID) = double(nearestSurvivor.ID);  % Ensure double type
                                undetectedSurvivors(nearestIdx).AssignedVehicle = vehicleID;
                                assignmentsMade = assignmentsMade + 1;
                                assignedSurvivorIDs = [assignedSurvivorIDs, nearestSurvivor.ID];
                                fprintf('Assigned survivor %d to UAV%d (distance: %.1fm)\n', ...
                                    nearestSurvivor.ID, i, minDist);
                            end
                            else
                                fprintf('Failed to plan path to survivor %d\n', nearestSurvivor.ID);
                            end
                        end
                    end
                end
            end
        
            % Then assign ground vehicles
            for i = 1:obj.numGroundVehicles
                if ~isempty(obj.groundPlatforms{i})
                    % ADD AT START OF GROUND VEHICLE LOOP
                    vehicleID = sprintf('Ground%d', i);
                    if obj.vehicleAssignments.isKey(vehicleID)
                        continue;  % Skip if vehicle already has an assignment
                    end
        
                    % Get current vehicle state and ensure it's valid
                    currentMotion = obj.groundPlatforms{i}.read();
                    if isempty(currentMotion) || all(currentMotion(1:3) == 0)
                        % Use initialization position if current position is invalid
                        spacing = obj.environment.dimensions(1) / (obj.numGroundVehicles + 1);
                        currentPos = [
                            i * spacing, ...
                            10, ...  % Offset from edge
                            0        % Ground level
                        ];
                        fprintf('Using initial position for Ground%d: [%.1f, %.1f, %.1f]\n', ...
                            i, currentPos(1), currentPos(2), currentPos(3));
                    else
                        currentPos = currentMotion(1:3);
                        fprintf('Current position for Ground%d: [%.1f, %.1f, %.1f]\n', ...
                            i, currentPos(1), currentPos(2), currentPos(3));
                    end
        
                    % Find nearest unassigned survivor for ground vehicle
                    minDist = inf;
                    nearestSurvivor = [];
                    nearestIdx = 0;
        
                    for s = 1:length(undetectedSurvivors)
                        if strcmp(undetectedSurvivors(s).Status, 'UNDETECTED')
                            % For ground vehicles, only consider horizontal distance
                            dist = norm(undetectedSurvivors(s).Position(1:2) - currentPos(1:2));
                            if dist < minDist
                                minDist = dist;
                                nearestSurvivor = undetectedSurvivors(s);
                                nearestIdx = s;
                            end
                        end
                    end
        
                    if ~isempty(nearestSurvivor)
                        goalPos = nearestSurvivor.Position;
                        goalPos(3) = 0;  % Ensure ground level
                        [success, path] = obj.planPath(currentPos, goalPos, false);
        
                        % REPLACEMENT BLOCK FOR GROUND VEHICLES
                        if success
                        obj.vehiclePaths{obj.numAerialVehicles + i} = path;
                        undetectedSurvivors(nearestIdx).Status = 'IN_PROGRESS';
                        vehicleID = char(sprintf('Ground%d', i));  % Ensure char type
                        if ~obj.vehicleAssignments.isKey(vehicleID)
                            obj.vehicleAssignments(vehicleID) = double(nearestSurvivor.ID);  % Ensure double type
                            undetectedSurvivors(nearestIdx).AssignedVehicle = vehicleID;
                            assignmentsMade = assignmentsMade + 1;
                            assignedSurvivorIDs = [assignedSurvivorIDs, nearestSurvivor.ID];
                            fprintf('Assigned survivor %d to ground vehicle %d (distance: %.1fm)\n', ...
                                nearestSurvivor.ID, i, minDist);
                        end
                        else
                            fprintf('Failed to plan path to survivor %d\n', nearestSurvivor.ID);
                        end
                    end
                end
            end
        
            % Print summary
            if assignmentsMade > 0
                fprintf('Assignment Summary: Made %d new assignments, Survivors: %s\n', ...
                    assignmentsMade, mat2str(assignedSurvivorIDs));
            else
                fprintf('No new assignments made\n');
            end
        end
        
        function updateVehiclePositions(obj)
            % Update vehicle positions based on planned paths
            for i = 1:obj.numAerialVehicles
                if ~isempty(obj.aerialPlatforms{i}) && ~isempty(obj.vehiclePaths{i})
                    if size(obj.vehiclePaths{i}, 1) > 1
                        nextWaypoint = obj.vehiclePaths{i}(1, :);
                        move(obj.aerialPlatforms{i}, nextWaypoint);
                        obj.vehiclePaths{i}(1,:) = []; % Remove reached waypoint
                    end
                end
            end
            
            for i = 1:obj.numGroundVehicles
                idx = obj.numAerialVehicles + i;
                if ~isempty(obj.groundPlatforms{i}) && ~isempty(obj.vehiclePaths{idx})
                    if size(obj.vehiclePaths{idx}, 1) > 1
                        nextWaypoint = obj.vehiclePaths{idx}(1, :);
                        move(obj.groundPlatforms{i}, nextWaypoint);
                        obj.vehiclePaths{idx}(1,:) = []; % Remove reached waypoint
                    end
                end
            end
        end
    end
end