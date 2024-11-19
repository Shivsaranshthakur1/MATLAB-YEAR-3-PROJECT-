% SurvivorManager.m
classdef SurvivorManager < handle
    properties
        Survivors       % Array of Survivor objects
        Environment    % Reference to environment
        Scenario      % Reference to UAV scenario
    end
    
    properties (Constant)
        SURVIVOR_RADIUS = 0.5    % Radius for survivor visualization (meters)
        SURVIVOR_HEIGHT = 1.8    % Height for survivor visualization (meters)
        MIN_SPACING = 5         % Minimum spacing between survivors (meters)
        MAX_PLACEMENT_ATTEMPTS = 100  % Maximum attempts to place a survivor
        MIN_BUILDING_DISTANCE = 3    % Minimum distance from buildings (meters)
    end
    
    methods
        function obj = SurvivorManager(environment)
            % Input validation
            if nargin < 1
                error('SurvivorManager:NoEnvironment', 'Environment must be provided');
            end
            if ~isa(environment, 'searchRescueEnvironment')
                error('SurvivorManager:InvalidEnvironment', ...
                    'Environment must be a searchRescueEnvironment object');
            end
            
            obj.Environment = environment;
            obj.Scenario = environment.scenario;
            obj.Survivors = [];
            
            % Verify scenario exists
            if isempty(obj.Scenario)
                error('SurvivorManager:NoScenario', 'UAV scenario not found in environment');
            end
        end
        
        function generateSurvivors(obj, numSurvivors)
            % Input validation
            if nargin < 2 || ~isnumeric(numSurvivors) || numSurvivors < 1
                error('SurvivorManager:InvalidNumber', ...
                    'Number of survivors must be a positive integer');
            end
            
            fprintf('Generating %d survivors...\n', numSurvivors);
            successfulPlacements = 0;
            totalAttempts = 0;
            
            while successfulPlacements < numSurvivors && totalAttempts < obj.MAX_PLACEMENT_ATTEMPTS * numSurvivors
                try
                    % Generate valid position
                    pos = obj.generateValidPosition();
                    
                    % Assign weighted random priority
                    priority = obj.generatePriority();
                    
                    % Create survivor object
                    survivor = Survivor(successfulPlacements + 1, pos, priority);
                    
                    % Try to add visual representation
                    try
                        obj.addSurvivorMesh(survivor);
                    catch meshError
                        warning('Failed to add survivor mesh: %s', meshError.message);
                        continue;
                    end
                    
                    % Add to survivors array
                    obj.Survivors = [obj.Survivors survivor];
                    
                    % Log success
                    successfulPlacements = successfulPlacements + 1;
                    fprintf('Generated Survivor %d: Position=[%.1f, %.1f, %.1f], Priority=%d\n', ...
                        survivor.ID, pos(1), pos(2), pos(3), priority);
                    
                catch genError
                    totalAttempts = totalAttempts + 1;
                    warning('Failed placement attempt: %s', genError.message);
                end
            end
            
            % Check if we succeeded in placing all survivors
            if successfulPlacements < numSurvivors
                warning('SurvivorManager:PlacementIncomplete', ...
                    'Only placed %d out of %d requested survivors after %d attempts', ...
                    successfulPlacements, numSurvivors, totalAttempts);
            else
                fprintf('Successfully placed all %d survivors\n', numSurvivors);
            end
        end
        
        function priority = generatePriority(~)
            % Generate weighted random priority
            r = rand();
            if r < 0.2
                priority = 1;     % 20% High priority
            elseif r < 0.7
                priority = 2;     % 50% Medium priority
            else
                priority = 3;     % 30% Low priority
            end
        end
        
        function pos = generateValidPosition(obj)
            % Get environment dimensions
            dims = obj.Environment.dimensions;
            
            % Try to find valid position
            for attempt = 1:obj.MAX_PLACEMENT_ATTEMPTS
                % Generate random position within bounds
                pos = [
                    rand() * dims(1),    % X
                    rand() * dims(2),    % Y
                    0                    % Z (ground level)
                ];
                
                % Check if position is valid
                if obj.isPositionValid(pos)
                    return;
                end
            end
            
            error('SurvivorManager:NoValidPosition', ...
                'Could not find valid position after %d attempts', ...
                obj.MAX_PLACEMENT_ATTEMPTS);
        end
        
        function valid = isPositionValid(obj, pos)
            try
                % Get buildings from environment
                buildings = obj.Environment.getBuildings();
                
                % Check if position is within environment bounds
                if ~obj.isWithinBounds(pos)
                    valid = false;
                    return;
                end
                
                % Check distance from buildings
                for i = 1:length(buildings)
                    if obj.isNearBuilding(pos, buildings(i))
                        valid = false;
                        return;
                    end
                end
                
                % Check distance from other survivors
                for i = 1:length(obj.Survivors)
                    otherPos = obj.Survivors(i).Position;
                    if norm(otherPos(1:2) - pos(1:2)) < obj.MIN_SPACING
                        valid = false;
                        return;
                    end
                end
                
                valid = true;
                
            catch e
                warning('Error checking position validity: %s', e.message);
                valid = false;
            end
        end
        
        function valid = isWithinBounds(obj, pos)
            % Check if position is within environment bounds with margin
            margin = obj.SURVIVOR_RADIUS + 1;  % 1m extra margin
            dims = obj.Environment.dimensions;
            
            valid = pos(1) >= margin && pos(1) <= (dims(1) - margin) && ...
                   pos(2) >= margin && pos(2) <= (dims(2) - margin) && ...
                   pos(3) >= 0;
        end
        
        function near = isNearBuilding(obj, pos, building)
            % Get building position and dimensions
            buildingPos = building.position;
            buildingDims = building.dimensions;
            
            % Calculate expanded bounds with safety margin
            minX = buildingPos(1) - obj.MIN_BUILDING_DISTANCE;
            maxX = buildingPos(1) + buildingDims(1) + obj.MIN_BUILDING_DISTANCE;
            minY = buildingPos(2) - obj.MIN_BUILDING_DISTANCE;
            maxY = buildingPos(2) + buildingDims(2) + obj.MIN_BUILDING_DISTANCE;
            
            % Check if position is within expanded bounds
            near = pos(1) >= minX && pos(1) <= maxX && ...
                   pos(2) >= minY && pos(2) <= maxY;
        end
        
         function addSurvivorMesh(obj, survivor)
            try
                % Validate survivor object
                if ~isa(survivor, 'Survivor')
                    error('Invalid survivor object');
                end
                
                % Extract position
                position = survivor.Position;
                
                % Create cylinder geometry in correct format
                % {[centerx centery radius], [zmin zmax]}
                geometry = {
                    [position(1) position(2) obj.SURVIVOR_RADIUS], ... % [x y radius]
                    [0 obj.SURVIVOR_HEIGHT]                           % [zmin zmax]
                };
                
                % Add cylinder mesh for survivor
                addMesh(obj.Scenario, 'cylinder', geometry, survivor.Color);
                
            catch e
                warning('Failed to add survivor mesh: %s', e.message);
            end
        end
        
        function survivors = getUndetectedSurvivors(obj)
            % Return list of undetected survivors with error checking
            try
                if isempty(obj.Survivors)
                    survivors = [];
                    return;
                end
                
                % Debug output
                fprintf('Checking %d total survivors\n', length(obj.Survivors));
                
                % Use strcmp for string comparison
                undetectedIdx = false(1, length(obj.Survivors));
                for i = 1:length(obj.Survivors)
                    undetectedIdx(i) = strcmp(obj.Survivors(i).Status, 'UNDETECTED');
                    % Debug output
                    fprintf('Survivor %d status: %s, Undetected? %d\n', ...
                        i, obj.Survivors(i).Status, undetectedIdx(i));
                end
                
                survivors = obj.Survivors(undetectedIdx);
                fprintf('Found %d undetected survivors after checking\n', length(survivors));
            catch e
                warning('Error getting undetected survivors: %s\n', e.message);
                survivors = [];
            end
        end
        
        function survivors = getHighPrioritySurvivors(obj)
            % Return list of high priority survivors with error checking
            try
                if isempty(obj.Survivors)
                    survivors = [];
                    return;
                end
                survivors = obj.Survivors([obj.Survivors.Priority] == 1);
            catch
                warning('Error getting high priority survivors. Returning empty array.');
                survivors = [];
            end
        end

        function showSurvivorStatus(obj)
            fprintf('\nCurrent Survivor Status:\n');
            for i = 1:length(obj.Survivors)
                s = obj.Survivors(i);
                fprintf('Survivor %d: Priority=%d, Status=%s, Position=[%.1f, %.1f, %.1f]\n', ...
                    s.ID, s.Priority, s.Status, s.Position(1), s.Position(2), s.Position(3));
            end
        end
        
        function reportDetection(obj, survivorID)
            try
                % Input validation
                if ~isnumeric(survivorID) || survivorID < 1
                    error('Invalid survivor ID');
                end
                
                % Find survivor
                survivorIdx = find([obj.Survivors.ID] == survivorID, 1);
                if isempty(survivorIdx)
                    warning('Survivor ID %d not found', survivorID);
                    return;
                end
                
                % Update status
                survivor = obj.Survivors(survivorIdx);
                if strcmp(survivor.Status, 'UNDETECTED')
                    survivor.Status = 'DETECTED';
                    fprintf('Survivor %d detected (Priority %d)\n', ...
                        survivorID, survivor.Priority);
                end
                
            catch e
                warning('Error reporting detection: %s', e.message);
            end
        end
    end
end