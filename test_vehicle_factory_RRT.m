% test_vehicle_factory_RRT.m
clear;
clc;
close all;
try
    fprintf('Testing updated VehicleFactory for RRT integration...\n\n');
    
    % Create a test scenario
    scenario = uavScenario('UpdateRate', 100, 'StopTime', Inf);
    
    % Test Aerial Vehicle Creation
    fprintf('1. Testing Aerial Vehicle Creation:\n');
    aerialPos = [50, 50, 30];
    aerialSpeed = 25;
    aerialVehicle = VehicleFactory.createAerialVehicle(...
        scenario, 'TestUAV', aerialPos, [1 0 0], aerialSpeed);
    
    % Verify initial aerial vehicle state
    initialAerialMotion = read(aerialVehicle);
    fprintf('Initial Aerial Position: [%.1f, %.1f, %.1f]\n', ...
        initialAerialMotion(1), initialAerialMotion(2), initialAerialMotion(3));
    fprintf('Initial Aerial Velocity: [%.1f, %.1f, %.1f]\n', ...
        initialAerialMotion(4), initialAerialMotion(5), initialAerialMotion(6));
    
    % Test Ground Vehicle Creation
    fprintf('\n2. Testing Ground Vehicle Creation:\n');
    groundPos = [100, 100, 0];
    groundSpeed = 15;
    groundVehicle = VehicleFactory.createGroundVehicle(...
        scenario, 'TestGround', groundPos, [0 0 1], groundSpeed);
    
    % Verify initial ground vehicle state
    initialGroundMotion = read(groundVehicle);
    fprintf('Initial Ground Position: [%.1f, %.1f, %.1f]\n', ...
        initialGroundMotion(1), initialGroundMotion(2), initialGroundMotion(3));
    fprintf('Initial Ground Velocity: [%.1f, %.1f, %.1f]\n', ...
        initialGroundMotion(4), initialGroundMotion(5), initialGroundMotion(6));
    
    % Test Position Updates
    fprintf('\n3. Testing Position Updates:\n');
    newAerialPos = [60, 60, 35];
    newGroundPos = [110, 110, 10]; % Should force z to 0 for ground vehicle
    
    fprintf('Moving aerial vehicle to [%.1f, %.1f, %.1f]\n', ...
        newAerialPos(1), newAerialPos(2), newAerialPos(3));
    VehicleFactory.updateVehiclePosition(aerialVehicle, newAerialPos);
    
    fprintf('Moving ground vehicle to [%.1f, %.1f, %.1f]\n', ...
        newGroundPos(1), newGroundPos(2), newGroundPos(3));
    VehicleFactory.updateVehiclePosition(groundVehicle, newGroundPos);
    
    % Verify final positions
    aerialCurrentMotion = read(aerialVehicle);
    groundCurrentMotion = read(groundVehicle);
    
    fprintf('\n4. Verifying Final States:\n');
    fprintf('Aerial Vehicle:\n');
    fprintf('  Position: [%.1f, %.1f, %.1f]\n', ...
        aerialCurrentMotion(1), aerialCurrentMotion(2), aerialCurrentMotion(3));
    fprintf('  Velocity: [%.1f, %.1f, %.1f]\n', ...
        aerialCurrentMotion(4), aerialCurrentMotion(5), aerialCurrentMotion(6));
    
    fprintf('Ground Vehicle:\n');
    fprintf('  Position: [%.1f, %.1f, %.1f]\n', ...
        groundCurrentMotion(1), groundCurrentMotion(2), groundCurrentMotion(3));
    fprintf('  Velocity: [%.1f, %.1f, %.1f]\n', ...
        groundCurrentMotion(4), groundCurrentMotion(5), groundCurrentMotion(6));
    
    % Verify ground vehicle z-coordinate is 0
    if groundCurrentMotion(3) ~= 0
        error('Ground vehicle z-coordinate is not 0');
    end
    
    % Verify speeds are maintained
    if abs(norm(aerialCurrentMotion(4:6)) - aerialSpeed) > 1e-6
        warning('Aerial vehicle speed changed');
    end
    if abs(norm(groundCurrentMotion(4:6)) - groundSpeed) > 1e-6
        warning('Ground vehicle speed changed');
    end
    
    % Visualize
    fprintf('\n5. Creating visualization...\n');
    figure('Name', 'Vehicle Factory Test');
    show3D(scenario);
    view(45, 30);
    grid on;
    
    fprintf('\nVehicleFactory test completed successfully!\n');
    
catch e
    fprintf('\nError occurred during VehicleFactory testing:\n');
    fprintf('Error message: %s\n', e.message);
    fprintf('Error location: %s\n', e.stack(1).name);
    fprintf('\nStack trace:\n');
    for i = 1:length(e.stack)
        fprintf('Stack %d: %s, Line %d\n', i, e.stack(i).name, e.stack(i).line);
    end
end