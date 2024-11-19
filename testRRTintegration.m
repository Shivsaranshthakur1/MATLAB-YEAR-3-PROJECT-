% test_rrt_integration.m
% Test script for verifying RRT integration
clear;
clc;
close all;

try
    % Define test environment dimensions
    dimensions = [300, 300, 100];
    fprintf('Setting up test environment [%d x %d x %d]...\n', dimensions(1), dimensions(2), dimensions(3));
    
    % Create environment
    environment = searchRescueEnvironment(dimensions);
    
    % Create controller with minimal vehicles for testing
    numAerial = 1;  % Start with just one UAV
    numGround = 0;  % No ground vehicles for initial test
    
    fprintf('Creating controller with %d aerial vehicles...\n', numAerial);
    controller = centralController_RRT(environment, numAerial, numGround);
    
    % Test basic path planning
    fprintf('\nTesting path planning...\n');
    startPos = [50, 50, 30];  % Starting position
    goalPos = [200, 200, 30]; % Goal position
    
    fprintf('Planning path from [%.1f, %.1f, %.1f] to [%.1f, %.1f, %.1f]\n', ...
        startPos(1), startPos(2), startPos(3), goalPos(1), goalPos(2), goalPos(3));
    
    [success, path] = controller.planPath(startPos, goalPos);
    
    if success
        % Visualize environment and path
        figure('Name', 'RRT Path Planning Test', 'Position', [100 100 1200 800]);
        
        % Show environment with path
        subplot(1,2,1);
        show(environment);
        hold on;
        
        % Plot path
        plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'Planned Path');
        plot3(startPos(1), startPos(2), startPos(3), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
        plot3(goalPos(1), goalPos(2), goalPos(3), 'ro', 'MarkerSize', 10, 'DisplayName', 'Goal');
        legend;
        title('Environment View with Path');
        
        % Show occupancy map view
        subplot(1,2,2);
        show(controller.occMap);
        hold on;
        plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2);
        plot3(startPos(1), startPos(2), startPos(3), 'go', 'MarkerSize', 10);
        plot3(goalPos(1), goalPos(2), goalPos(3), 'ro', 'MarkerSize', 10);
        title('Occupancy Map View');
        view(45, 30);
        
        fprintf('\nPath planning successful!\n');
        fprintf('Path contains %d waypoints\n', size(path,1));
    else
        fprintf('\nPath planning failed!\n');
    end
    
catch e
    fprintf('\nError occurred!\n');
    fprintf('Error message: %s\n', e.message);
    fprintf('Error location: %s\n', e.stack(1).name);
    fprintf('\nStack trace:\n');
    for i = 1:length(e.stack)
        fprintf('Stack %d: %s, Line %d\n', i, e.stack(i).name, e.stack(i).line);
    end
end