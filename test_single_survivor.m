% test_survivor.m
clear;
clc;
close all;

try
    % Create basic scenario
    scenario = uavScenario('UpdateRate', 100);
    
    % Add ground plane
    addMesh(scenario, 'Polygon', ...
        {[-100 -100; 100 -100; 100 100; -100 100], [-1 0]}, ...
        [0.7 0.7 0.7]);
    
    % Add single survivor (test cylinder)
    position = [0 0];  % x,y position
    radius = 0.5;      % radius in meters
    height = [0 1.8];  % height range [min max]
    
    disp('Testing cylinder mesh creation...');
    disp(['Position: [' num2str(position) ']']);
    disp(['Radius: ' num2str(radius)]);
    disp(['Height range: [' num2str(height) ']']);
    
    % Try to add cylinder with different geometry format
    addMesh(scenario, 'cylinder', ...
        {[position radius], height}, ... % Format from documentation
        [1 0 0]);  % Red color
    
    % Show result
    figure;
    show3D(scenario);
    view(45, 30);
    grid on;
    
catch e
    fprintf('Error: %s\n', e.message);
    disp(e.stack);
end