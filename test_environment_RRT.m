% test_environment_RRT.m
% Test script for RRT-enhanced search and rescue environment
clear;
clc;
close all;

try
    % Define search area dimensions [length width height] in meters
    dimensions = [300, 300, 100];
    
    % Create and initialize environment
    disp('Initializing search and rescue environment...');
    environment = searchRescueEnvironment(dimensions);
    
    % Verify occupancy map creation
    disp('Verifying 3D occupancy map...');
    occMap = environment.getOccupancyMap();
    fprintf('Occupancy map properties:\n');
    fprintf('- Resolution: %.2f cells/meter\n', occMap.Resolution);
    
    % Check some key positions in the map
    groundPoint = [150 150 0];  % Should be free space
    buildingPoint = [50 50 20]; % Should be occupied (center of first building)
    
    groundOcc = getOccupancy(occMap, groundPoint);
    buildingOcc = getOccupancy(occMap, buildingPoint);
    
    fprintf('- Ground point occupancy (should be ~0): %.2f\n', groundOcc);
    fprintf('- Building point occupancy (should be ~1): %.2f\n', buildingOcc);
    
    % Start with fewer vehicles for initial RRT testing
    numAerialVehicles = 2; % Reduced number for testing
    numGroundVehicles = 1; % Reduced number for testing
    
    % Create and initialize RRT controller
    disp('Initializing RRT-enhanced central controller...');
    controller = centralController_RRT(environment, numAerialVehicles, numGroundVehicles);
    
    % Test RRT path planning
    disp('Testing RRT path planning capabilities...');
    startPos = [30, 30, 30];  % Test start position
    goalPos = [250, 250, 30]; % Test goal position
    
    fprintf('Planning test path from [%.1f, %.1f, %.1f] to [%.1f, %.1f, %.1f]\n', ...
        startPos(1), startPos(2), startPos(3), goalPos(1), goalPos(2), goalPos(3));
    
    [success, testPath] = controller.planPath(startPos, goalPos);
    
    if success
        fprintf('Test path planning successful!\n');
        fprintf('Path contains %d waypoints\n', size(testPath,1));
    else
        fprintf('Test path planning failed!\n');
    end
    
    % Create visualization
    disp('Creating environment visualization...');
    figure('Name', 'RRT-Enhanced Environment', 'Position', [100 100 1200 800]);
    
    % Show scenario with RRT path
    subplot(1,2,1);
    show(environment);
    hold on;
    if success
        plot3(testPath(:,1), testPath(:,2), testPath(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'RRT Test Path');
        plot3(startPos(1), startPos(2), startPos(3), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
        plot3(goalPos(1), goalPos(2), goalPos(3), 'ro', 'MarkerSize', 10, 'DisplayName', 'Goal');
        legend;
    end
    title('Scenario View with RRT Path');
    
    % Show occupancy map
    subplot(1,2,2);
    show(occMap);
    hold on;
    if success
        plot3(testPath(:,1), testPath(:,2), testPath(:,3), 'r-', 'LineWidth', 2);
        plot3(startPos(1), startPos(2), startPos(3), 'go', 'MarkerSize', 10);
        plot3(goalPos(1), goalPos(2), goalPos(3), 'ro', 'MarkerSize', 10);
    end
    title('Occupancy Map View');
    view(45, 30);
    
    % Start the mission
    disp('Starting mission with RRT path planning...');
    fprintf('Search and rescue mission starting with:\n');
    fprintf('- %d Aerial Vehicles (red quadrotors)\n', numAerialVehicles);
    fprintf('- %d Ground Vehicles (blue cuboids)\n', numGroundVehicles);
    fprintf('- %d Survivors (color-coded cylinders)\n', length(controller.survivorManager.Survivors));
    fprintf('\nSurvivor color coding:\n');
    fprintf('- Red: High Priority\n');
    fprintf('- Orange: Medium Priority\n');
    fprintf('- Yellow: Low Priority\n');
    fprintf('\nClose the figure window to end simulation.\n\n');
    
    % Create mission visualization
    figure('Name', 'Search and Rescue Mission');
    
    if controller.startMission()
        disp('Mission completed successfully.');
    else
        disp('Mission failed.');
    end
    
catch e
    % Error handling
    fprintf('\nERROR OCCURRED!\n');
    fprintf('Error message: %s\n', e.message);
    fprintf('Error identifier: %s\n\n', e.identifier);
    fprintf('Stack trace:\n');
    for k = 1:length(e.stack)
        fprintf('File: %s\n', e.stack(k).file);
        fprintf('Function: %s\n', e.stack(k).name);
        fprintf('Line: %d\n\n', e.stack(k).line);
    end
end