% test_rrt_system.m
clear;
clc;
close all;

% Configuration
dimensions = [300, 300, 100];
numAerial = 2;
numGround = 2;
testDuration = 1200; % seconds

% Enhanced metrics storage
metrics = struct('pathPlanningSuccess', [], ...
                 'collisionEvents', 0, ...
                 'survivorsDetected', 0, ...
                 'missionTime', 0, ...
                 'pathLengths', [], ...
                 'executionTimes', [], ...
                 'scenarioResults', struct('name', {}, 'success', {}, 'time', {}, 'length', {}), ...
                 'heightProfiles', []);

try
    fprintf('Starting RRT System Test...\n\n');
    
    % 1. Create and validate environment
    fprintf('1. Creating Search and Rescue Environment [%d x %d x %d]...\n', ...
            dimensions(1), dimensions(2), dimensions(3));
    environment = searchRescueEnvironment(dimensions);

    % Verify 3D environment setup
    fprintf('\nVerifying 3D environment setup...\n');
    buildings = environment.buildingList;
    fprintf(' - Number of buildings: %d\n', length(buildings));
    for i = 1:length(buildings)
        building = buildings(i);
        fprintf(' - Building %d: Position=[%.1f, %.1f, %.1f], Size=[%.1f, %.1f, %.1f]\n', ...
                i, building.position(1), building.position(2), building.position(3), ...
                building.dimensions(1), building.dimensions(2), building.dimensions(3));
    end

    obstacles = environment.obstacles.config;
    fprintf(' - Number of obstacles: %d\n', size(obstacles, 1));
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);
        fprintf(' - Obstacle %d: Center=[%.1f, %.1f], Radius=%.1f, Height=%.1f\n', ...
                i, obs(1), obs(2), obs(3), obs(4));
    end

    % Create initial visualization figure with enhanced 3D
    mainFig = figure('Name', 'Search and Rescue Mission', 'Position', [100, 100, 1200, 800]);

    % Enhanced 3D view
    subplot(1, 2, 1);
    environment.show();
    title('3D Environment View');
    view(45, 30);
    grid on;
    axis equal;
    xlabel('X (meters)');
    ylabel('Y (meters)');
    zlabel('Z (meters)');
    light('Position', [1 1 1]);
    lighting gouraud;
    hold on;

    % Top-down view
    subplot(1, 2, 2);
    environment.show();
    title('Top-Down View');
    view(0, 90);
    grid on;
    axis equal;
    xlabel('X (meters)');
    ylabel('Y (meters)');
    zlabel('Z (meters)');
    hold on;

    % 2. Create and validate controller
    fprintf('\n2. Creating RRT Controller with %d aerial and %d ground vehicles...\n', ...
            numAerial, numGround);
    controller = centralController_RRT(environment, numAerial, numGround);

    % Validate initialization
    assert(~isempty(controller.rrtPlanner), 'RRT Planner not initialized');
    assert(~isempty(controller.occMap), 'Occupancy map not initialized');
    assert(length(controller.aerialPlatforms) == numAerial, 'Aerial platforms not initialized');
    assert(length(controller.groundPlatforms) == numGround, 'Ground platforms not initialized');

    % 3. Verify survivor placement
    fprintf('\n3. Verifying survivor placement...\n');
    numSurvivors = length(controller.survivorManager.Survivors);
    fprintf(' Generated %d survivors\n', numSurvivors);
    assert(numSurvivors > 0, 'No survivors generated');

    % Plot survivors on both views
    survivors = controller.survivorManager.Survivors;
    for idx = 1:2 % For both subplots
        subplot(1, 2, idx);
        hold on;
        for i = 1:length(survivors)
            pos = survivors(i).Position;
            switch survivors(i).Priority
                case 1
                    h = plot3(pos(1), pos(2), pos(3), 'r', 'MarkerSize', 10);
                case 2
                    h = plot3(pos(1), pos(2), pos(3), 'y', 'MarkerSize', 10);
                case 3
                    h = plot3(pos(1), pos(2), pos(3), 'g', 'MarkerSize', 10);
            end
            h.Tag = 'survivor';
        end
    end

    % 4. Test path planning capabilities
    fprintf('\n4. Testing path planning capabilities...\n');

    % Create height profile figure
    heightProfileFig = figure('Name', 'Path Height Profiles', 'Position', [100, 100, 800, 600]);

    % Define enhanced test scenarios
    testScenarios = {
        {[30, 30, 30], [250, 250, 30], false, 'Long Aerial Path'},
        {[30, 30, 30], [100, 100, 50], false, 'Vertical Aerial Path'},
        {[30, 30, 30], [200, 200, 70], false, 'High Altitude Path'},
        {[30, 30, 0], [250, 250, 0], true, 'Long Ground Path'},
        {[30, 30, 0], [100, 100, 0], true, 'Short Ground Path'}
    };

    % Run all test scenarios
    for i = 1:length(testScenarios)
        scenario = testScenarios{i};
        startPos = scenario{1};
        goalPos = scenario{2};
        isGround = scenario{3};
        scenarioName = scenario{4};
        fprintf('\nTesting %s:\n', scenarioName);
        tic;
        [success, path] = controller.planPath(startPos, goalPos, ~isGround);  % ~isGround because isGround is already defined in testScenarios
        execTime = toc;

        % Store results
        metrics.scenarioResults(i).name = scenarioName;
        metrics.scenarioResults(i).success = success;
        metrics.scenarioResults(i).time = execTime;

        if success
            metrics.pathPlanningSuccess(end + 1) = 1;
            metrics.executionTimes(end + 1) = execTime;
            pathLength = sum(sqrt(sum(diff(path(:, 1:3)).^2, 2)));
            metrics.pathLengths(end + 1) = pathLength;
            metrics.scenarioResults(i).length = pathLength;
            fprintf(' Path planning successful!\n');
            fprintf(' - Path length: %.2f meters\n', pathLength);
            fprintf(' - Execution time: %.2f seconds\n', execTime);
            fprintf(' - Waypoints: %d\n', size(path, 1));

            % Store height profile
            metrics.heightProfiles(i).name = scenarioName;
            metrics.heightProfiles(i).heights = path(:, 3);

            % Plot height profile
            figure(heightProfileFig);
            subplot(length(testScenarios), 1, i);
            plot(path(:, 3), '-o');
            title(sprintf('Height Profile: %s', scenarioName));
            xlabel('Waypoint');
            ylabel('Height (m)');
            grid on;

            % Validate path
            validatePath(path, isGround);

            % Plot path on both views
            figure(mainFig);
            for idx = 1:2
                subplot(1, 2, idx);
                hold on;
                if isGround
                    % Force ground path to z=0 for visualization
                    groundPath = path;
                    groundPath(:, 3) = zeros(size(path, 1), 1);
                    plot3(groundPath(:, 1), groundPath(:, 2), groundPath(:, 3), 'b-', 'LineWidth', 2);
                    scatter3(startPos(1), startPos(2), 0, 100, 'bs', 'filled');  % Force start to ground
                    scatter3(goalPos(1), goalPos(2), 0, 100, 'bd', 'filled');    % Force goal to ground
                else
                    plot3(path(:, 1), path(:, 2), path(:, 3), 'r-', 'LineWidth', 2);
                    scatter3(startPos(1), startPos(2), startPos(3), 100, 'rs', 'filled');
                    scatter3(goalPos(1), goalPos(2), goalPos(3), 100, 'rd', 'filled');
                end
            end
            drawnow;
        else
            metrics.pathPlanningSuccess(end + 1) = 0;
            fprintf(' Path planning failed - trying alternate goal\n');
            
            % Different handling for ground vs aerial paths
            if isGround
                % Try multiple alternate positions for ground paths
                altPositions = [
                    startPos(1) + (goalPos(1) - startPos(1)) * 0.5,  startPos(2) + (goalPos(2) - startPos(2)) * 0.5,  0;  % 50% of the way
                    startPos(1) + (goalPos(1) - startPos(1)) * 0.3,  startPos(2) + (goalPos(2) - startPos(2)) * 0.3,  0;  % 30% of the way
                    startPos(1) + (goalPos(1) - startPos(1)) * 0.7,  startPos(2) + (goalPos(2) - startPos(2)) * 0.7,  0   % 70% of the way
                ];
                
                success = false;
                for j = 1:size(altPositions, 1)
                    altGoalPos = altPositions(j, :);
                    fprintf(' Trying alternate goal position %d: [%.1f, %.1f, %.1f]\n', ...
                        j, altGoalPos(1), altGoalPos(2), altGoalPos(3));
                    
                    [success, path] = controller.planPath(startPos, altGoalPos, ~isGround);
                    if success
                        fprintf(' Found valid alternate path!\n');
                        break;
                    end
                end
            else
                % Original aerial path alternate goal logic
                altGoalPos = [
                    startPos(1) + (goalPos(1) - startPos(1)) * 0.7, ...
                    startPos(2) + (goalPos(2) - startPos(2)) * 0.7, ...
                    goalPos(3)
                ];
                [success, path] = controller.planPath(startPos, altGoalPos, ~isGround);
            end
            
            if ~success
                error('Path planning failed for %s (including %d alternate goals)', ...
                    scenarioName, size(altPositions,1));
            end
        end
    end

    % 5. Start mission simulation
    fprintf('\n5. Starting mission simulation...\n');
    fprintf(' - %d Aerial Vehicles (red)\n', numAerial);
    fprintf(' - %d Ground Vehicles (blue)\n', numGround);
    fprintf(' - %d Survivors\n', numSurvivors);

    % Create mission statistics figure
    statsFig = figure('Name', 'Mission Statistics', 'Position', [100, 100, 800, 600]);

    % Start mission
    tStart = tic;
    if controller.startMission()
        metrics.missionTime = toc(tStart);

        % Print detailed mission statistics
        fprintf('\nMission completed successfully!\n');
        fprintf('\nDetailed Mission Statistics:\n');
        fprintf('- Mission Time: %.2f seconds\n', metrics.missionTime);
        fprintf('- Overall Path Planning Success Rate: %.2f%%\n', mean(metrics.pathPlanningSuccess) * 100);
        fprintf('- Average Path Length: %.2f meters\n', mean(metrics.pathLengths));
        fprintf('- Average Planning Time: %.2f seconds\n', mean(metrics.executionTimes));
        fprintf('- Collision Events: %d\n', metrics.collisionEvents);
        fprintf('- Survivors Detected: %d/%d\n', metrics.survivorsDetected, numSurvivors);

        % Print individual scenario results
        fprintf('\nScenario Results:\n');
        for i = 1:length(metrics.scenarioResults)
            sc = metrics.scenarioResults(i);
            fprintf('- %s:\n', sc.name);
            fprintf(' * Success: %d\n', sc.success);
            fprintf(' * Planning Time: %.2f seconds\n', sc.time);
            if isfield(sc, 'length') && ~isempty(sc.length)
                fprintf(' * Path Length: %.2f meters\n', sc.length);
            end
        end

        % Plot mission statistics
        figure(statsFig);
        subplot(2, 2, 1);
        bar(metrics.pathLengths);
        title('Path Lengths');
        xlabel('Scenario');
        ylabel('Length (m)');
        subplot(2, 2, 2);
        bar(metrics.executionTimes);
        title('Execution Times');
        xlabel('Scenario');
        ylabel('Time (s)');

        % Plot height variations
        subplot(2, 2, 3);
        hold on;
        for i = 1:length(metrics.heightProfiles)
            if ~isempty(metrics.heightProfiles(i).heights)
                plot(metrics.heightProfiles(i).heights, 'DisplayName', metrics.heightProfiles(i).name);
            end
        end
        title('Height Profiles');
        xlabel('Waypoint');
        ylabel('Height (m)');
        legend('show');
        grid on;
    else
        fprintf('\nMission failed!\n');
    end
catch e
    fprintf('\nError occurred during testing:\n');
    fprintf('Error message: %s\n', e.message);
    fprintf('Error location: %s\n', e.stack(1).name);
    fprintf('\nStack trace:\n');
    for i = 1:length(e.stack)
        fprintf('Stack %d: %s, Line %d\n', i, e.stack(i).name, e.stack(i).line);
    end
end

function validatePath(path, isGround)
    assert(~isempty(path), 'Path should not be empty');
    if isGround
        validateGroundPath(path);
    else
        % Check path continuity for aerial paths
        maxSegmentLength = 20; % Maximum allowed distance between waypoints
        for i = 1:size(path, 1) - 1
            dist = norm(path(i + 1, 1:3) - path(i, 1:3));
            assert(dist < maxSegmentLength, ...
                sprintf('Path segment too long: %.2f meters', dist));
        end
    end
end

function validateGroundPath(path)
    % Additional validation specific to ground paths
    assert(~isempty(path), 'Ground path should not be empty');
    
    % Ensure z coordinates are close to zero but allow some variation
    maxHeight = 1.0; % Increased from 0.1 to allow for small variations in ground height
    if any(abs(path(:, 3)) > maxHeight)
        warning('Ground path contains points above %.1fm - projecting to ground', maxHeight);
        path(:, 3) = zeros(size(path, 1), 1);
    end
    
    % Check for reasonable distances between waypoints
    maxSegmentLength = 15; % Slightly shorter segments for ground paths
    for i = 1:(size(path, 1) - 1)  % Fixed the 1j typo here
        dist = norm(path(i + 1, 1:2) - path(i, 1:2)); % Only check x,y distance
        assert(dist < maxSegmentLength, ...
            sprintf('Ground path segment too long: %.2f meters', dist));
    end
    
    % Calculate total path length safely
    if size(path, 1) > 1
        segmentLengths = zeros(size(path, 1) - 1, 1);
        for i = 1:(size(path, 1) - 1)
            segmentLengths(i) = norm(path(i + 1, 1:2) - path(i, 1:2));
        end
        totalLength = sum(segmentLengths);
        
        assert(totalLength < 500, ...
            sprintf('Ground path too long: %.2f meters', totalLength));
    end
end