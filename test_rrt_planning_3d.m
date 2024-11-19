% test_rrt_planning_3d.m
% Test script for 3D RRT path planning
clear;
clc;

try
    fprintf('Starting RRT 3D path planning test...\n\n');
    
    % Define test environment dimensions
    dimensions = [300, 300, 100];  
    fprintf('Step 1: Dimensions set to [%d, %d, %d]\n', dimensions(1), dimensions(2), dimensions(3));
    
    % Initialize 3D RRT planner
    fprintf('Step 2: Attempting to initialize RRT planner...\n');
    planner = initializeRRTPlanner3D(dimensions);
    fprintf('Step 3: RRT planner initialized successfully\n');
    
    % Access occupancy map from validator
    fprintf('Step 4: Accessing occupancy map from validator...\n');
    map = planner.StateValidator.Map;
    fprintf('Step 5: Map accessed successfully\n');
    
    % Add test obstacles
    fprintf('Step 6: Attempting to add test obstacles...\n');
    try
        % Add single test obstacle first for testing
        createBoxObstacle(map, [100 100 30], [20 20 30]);  % Single central obstacle
        fprintf('First obstacle added successfully\n');
        
        % Verify map state after adding obstacle
        fprintf('Checking map state...\n');
        occupiedPoints = checkOccupancy(map, [100 100 30]);
        fprintf('Occupancy at obstacle center: %.2f\n', occupiedPoints);
        
    catch obstacleError
        fprintf('Error adding obstacles: %s\n', obstacleError.message);
        fprintf('Stack trace for obstacle error:\n');
        for i = 1:length(obstacleError.stack)
            fprintf(' - %s: Line %d\n', obstacleError.stack(i).name, obstacleError.stack(i).line);
        end
        rethrow(obstacleError);
    end
    
    % Define start and goal states [x y z qw qx qy qz]
    fprintf('\nStep 7: Setting up start and goal states...\n');
    start = [50 50 20 1 0 0 0];     % Moved away from edges
    goal = [200 200 50 1 0 0 0];    % More central position
    
    fprintf('Start state: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n', start);
    fprintf('Goal state: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n', goal);
    
    % Verify start and goal states
    fprintf('\nStep 8: Verifying start and goal states...\n');
    
    % Check start state
    fprintf('Checking start state validity...\n');
    [startValid] = isStateValid(planner.StateValidator, start);
    if ~startValid
        fprintf('Start state [%.1f, %.1f, %.1f] is invalid.\n', start(1), start(2), start(3));
        % Get occupancy at start position
        startOcc = getOccupancy(planner.StateValidator.Map, [start(1) start(2) start(3)]);
        fprintf('Occupancy at start position: %.2f\n', startOcc);
        error('Start state is not valid. Please choose a different starting position.');
    else
        fprintf('Start state is valid.\n');
    end
    
    % Check goal state
    fprintf('Checking goal state validity...\n');
    [goalValid] = isStateValid(planner.StateValidator, goal);
    if ~goalValid
        fprintf('Goal state [%.1f, %.1f, %.1f] is invalid.\n', goal(1), goal(2), goal(3));
        % Get occupancy at goal position
        goalOcc = getOccupancy(planner.StateValidator.Map, [goal(1) goal(2) goal(3)]);
        fprintf('Occupancy at goal position: %.2f\n', goalOcc);
        error('Goal state is not valid. Please choose a different goal position.');
    else
        fprintf('Goal state is valid.\n');
    end
    
    % Plan path
    fprintf('\nStep 9: Planning path...\n');
    [pathObj, solnInfo] = plan(planner, start, goal);
    fprintf('Path planning complete!\n');
    
    % Visualization
    fprintf('\nStep 10: Creating visualization...\n');
    figure('Name', '3D RRT Path Planning', 'NumberTitle', 'off', 'Position', [100 100 1200 800]);
    
    % Show the occupancy map
    subplot(1,2,1)
    show(map)
    title('3D Environment with Path')
    hold on
    
    % Plot the RRT tree expansion
    tree = solnInfo.TreeData;
    plot3(tree(:,1), tree(:,2), tree(:,3), 'b.', 'MarkerSize', 1, 'DisplayName', 'RRT Tree');
    
    % Plot the final path
    path = pathObj.States;
    plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'Planned Path');
    
    % Plot start and goal positions
    scatter3(start(1), start(2), start(3), 100, 'g', 'filled', 'DisplayName', 'Start');
    scatter3(goal(1), goal(2), goal(3), 100, 'r', 'filled', 'DisplayName', 'Goal');
    
    % Customize the plot
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on
    view(45, 30);
    legend;
    
    % Create a second view from top
    subplot(1,2,2)
    show(map)
    title('Top View')
    hold on
    plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'Planned Path');
    scatter3(start(1), start(2), start(3), 100, 'g', 'filled', 'DisplayName', 'Start');
    scatter3(goal(1), goal(2), goal(3), 100, 'r', 'filled', 'DisplayName', 'Goal');
    view(0, 90);  % Top-down view
    grid on
    legend;
    
    % Print path statistics
    fprintf('\nPath Statistics:\n');
    fprintf('Path length: %.2f meters\n', pathLength(pathObj));
    fprintf('Number of waypoints: %d\n', size(pathObj.States, 1));
    fprintf('Average distance between waypoints: %.2f meters\n', ...
        pathLength(pathObj)/(size(pathObj.States, 1)-1));
    
    % Verify path validity
    fprintf('\nVerifying path validity...\n');
    isValid = true;
    for i = 1:size(path,1)-1
        [motionValid, ~] = isMotionValid(planner.StateValidator, path(i,:), path(i+1,:));
        if ~motionValid
            isValid = false;
            fprintf('Warning: Invalid motion detected between waypoints %d and %d\n', i, i+1);
        end
    end
    
    if isValid
        fprintf('All path segments are valid!\n');
    end
    
    fprintf('\nTest completed successfully!\n');
    
catch e
    fprintf('\nError occurred!\n');
    fprintf('Error message: %s\n', e.message);
    fprintf('Error location: %s\n', e.stack(1).name);
    fprintf('\nFull stack trace:\n');
    for i = 1:length(e.stack)
        fprintf('Stack %d: %s, Line %d\n', i, e.stack(i).name, e.stack(i).line);
    end
end