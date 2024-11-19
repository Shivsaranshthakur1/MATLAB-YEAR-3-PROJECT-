% searchRescueEnvironment.m
classdef searchRescueEnvironment < handle
    properties
        scenario        % UAV scenario object
        dimensions     % Search area dimensions [length width height]
        buildingData   % Structure containing building information
        obstacles      % Structure containing obstacle information
        buildingList   % Array of building structures with positions and dimensions
        occupancyMap   % 3D occupancy map for path planning
    end
    
    methods
        function obj = searchRescueEnvironment(dimensions)
            % Initialize the environment with dimensions and 3D occupancy map
            obj.dimensions = dimensions;
            obj.scenario = uavScenario('UpdateRate', 100, ...
                                     'StopTime', Inf, ...
                                     'ReferenceLocation', [0 0 0]);
            
            % Initialize 3D occupancy map
            resolution = 1; % 1 cell per meter
            obj.occupancyMap = occupancyMap3D(resolution);
            
            % Initialize all space as free
            obj.initializeFreeSpace();
            
            % Create environment components
            obj.createGround();
            obj.createBuildings();
            obj.createObstacles();
        end
        
        function initializeFreeSpace(obj)
            % Initialize entire space as free
            fprintf('Initializing free space in environment...\n');
            [X, Y, Z] = meshgrid(0:obj.occupancyMap.Resolution:obj.dimensions(1), ...
                                0:obj.occupancyMap.Resolution:obj.dimensions(2), ...
                                0:obj.occupancyMap.Resolution:obj.dimensions(3));
            points = [X(:) Y(:) Z(:)];
            setOccupancy(obj.occupancyMap, points, zeros(size(points,1), 1));
        end
        
        function createGround(obj)
            % Create ground plane with specified dimensions
            groundVertices = [
                0            0;
                obj.dimensions(1) 0;
                obj.dimensions(1) obj.dimensions(2);
                0            obj.dimensions(2)
            ];
            
            % Add ground mesh (slight depression for visual effect)
            addMesh(obj.scenario, 'Polygon', ...
                {groundVertices, [-1 0]}, ... % vertices and height range
                [0.7 0.7 0.7]); % grey color
        end
        
        function createBuildings(obj)
            % Define building configurations [x, y, width, length, height]
            buildingConfigs = [
                % Tall buildings (representing office/apartment blocks)
                [50,  50,  20, 30, 40];   % Central tall building
                [100, 80,  25, 25, 35];   % Office block
                [150, 60,  15, 45, 25];   % Long building
                [200, 120, 30, 30, 45];   % High-rise
                
                % Medium buildings
                [75,  150, 20, 20, 20];   % Square building
                [120, 180, 15, 25, 15];   % Small office
                
                % Small buildings/structures
                [180, 40,  10, 10, 10];   % Small structure
                [220, 90,  12, 12, 12]    % Small structure
            ];
            
            % Initialize building list
            obj.buildingList = struct('position', cell(1, size(buildingConfigs, 1)), ...
                                    'dimensions', cell(1, size(buildingConfigs, 1)));
            
            % Create each building
            for i = 1:size(buildingConfigs, 1)
                config = buildingConfigs(i, :);
                vertices = obj.createBuildingVertices(config);
                
                % Store building data
                obj.buildingList(i).position = [config(1), config(2), 0];
                obj.buildingList(i).dimensions = [config(3), config(4), config(5)];
                
                % Add building mesh to scenario
                addMesh(obj.scenario, 'Polygon', ...
                    {vertices, [0 config(5)]}, ... % vertices and height
                    [0.8 0.8 0.8]); % light grey color
                
                % Add building to occupancy map
                obj.addBuildingToOccupancyMap(obj.buildingList(i));
            end
        end
        
        function vertices = createBuildingVertices(~, config)
            % Extract building parameters
            x = config(1); y = config(2);
            width = config(3); length = config(4);
            
            % Create vertices for building footprint
            vertices = [
                x          y;
                x + width  y;
                x + width  y + length;
                x          y + length
            ];
        end
        
        function addBuildingToOccupancyMap(obj, building)
            % Add a building to the 3D occupancy map
            % building should have: position [x y z] and dimensions [length width height]
            
            % Create ranges for the building
            xRange = (building.position(1)):obj.occupancyMap.Resolution:(building.position(1) + building.dimensions(1));
            yRange = (building.position(2)):obj.occupancyMap.Resolution:(building.position(2) + building.dimensions(2));
            zRange = (building.position(3)):obj.occupancyMap.Resolution:(building.position(3) + building.dimensions(3));
            
            % Create grid points for the building
            [X, Y, Z] = meshgrid(xRange, yRange, zRange);
            points = [X(:) Y(:) Z(:)];
            
            % Set building space as occupied
            setOccupancy(obj.occupancyMap, points, ones(size(points,1), 1));
        end
        
        function createObstacles(obj)
            % Define obstacle configurations [x, y, radius, height]
            obstacleConfigs = [
                % Tall cylindrical obstacles (e.g., towers, poles)
                [180, 40,  3, 50];    % Communication tower
                [90,  120, 2, 30];    % Utility pole
                [220, 180, 4, 40];    % Water tower
                
                % Medium height obstacles
                [140, 70,  2, 20];    % Street light
                [160, 150, 2, 20];    % Street light
                [60,  200, 2, 20];    % Street light
            ];
            
            % Store obstacle data
            obj.obstacles = struct('config', obstacleConfigs);
            
            % Create each obstacle
            for i = 1:size(obstacleConfigs, 1)
                config = obstacleConfigs(i, :);
                
                % Add cylinder mesh to scenario
                addMesh(obj.scenario, 'Cylinder', ...
                    {[config(1) config(2) config(3)], [0 config(4)]}, ... % center and height range
                    [0.6 0.6 0.6]); % darker grey for contrast
                
                % Add cylinder to occupancy map
                [X, Y, Z] = cylinder(config(3), 20); % 20 points for cylinder approximation
                X = X * config(3) + config(1);
                Y = Y * config(3) + config(2);
                Z = Z * config(4);
                points = [X(:) Y(:) Z(:)];
                setOccupancy(obj.occupancyMap, points, ones(size(points,1), 1));
            end
        end
        
        function buildings = getBuildings(obj)
            % Return the list of buildings
            buildings = obj.buildingList;
        end
        
        function map = getOccupancyMap(obj)
            % Return the occupancy map for path planning
            map = obj.occupancyMap;
        end
        
       function show(obj)
            % Create figure and show scenario
            ax = gca;
            hold(ax, 'on');
            
            % Show occupancy map
            show(obj.occupancyMap, 'Parent', ax);
            
            % Plot buildings as 3D boxes
            for i = 1:length(obj.buildingList)
                building = obj.buildingList(i);
                pos = building.position;
                dims = building.dimensions;
                
                % Create building vertices
                [X,Y,Z] = obj.createBuildingBox(pos, dims);
                
                % Plot building faces
                fill3(X(:,[1 2 3 4 1])', Y(:,[1 2 3 4 1])', Z(:,[1 2 3 4 1])', [0.8 0.8 0.8], 'EdgeColor', [0.5 0.5 0.5]);
                fill3(X(:,[5 6 7 8 5])', Y(:,[5 6 7 8 5])', Z(:,[5 6 7 8 5])', [0.8 0.8 0.8], 'EdgeColor', [0.5 0.5 0.5]);
                fill3(X(:,[1 5 8 4 1])', Y(:,[1 5 8 4 1])', Z(:,[1 5 8 4 1])', [0.7 0.7 0.7], 'EdgeColor', [0.5 0.5 0.5]);
                fill3(X(:,[2 6 7 3 2])', Y(:,[2 6 7 3 2])', Z(:,[2 6 7 3 2])', [0.7 0.7 0.7], 'EdgeColor', [0.5 0.5 0.5]);
                fill3(X(:,[4 8 7 3 4])', Y(:,[4 8 7 3 4])', Z(:,[4 8 7 3 4])', [0.9 0.9 0.9], 'EdgeColor', [0.5 0.5 0.5]);
                fill3(X(:,[1 5 6 2 1])', Y(:,[1 5 6 2 1])', Z(:,[1 5 6 2 1])', [0.9 0.9 0.9], 'EdgeColor', [0.5 0.5 0.5]);
            end
            
            % Plot obstacles as cylinders
            for i = 1:size(obj.obstacles.config, 1)
                config = obj.obstacles.config(i,:);
                [X,Y,Z] = cylinder(config(3), 20);
                X = X * config(3) + config(1);
                Y = Y * config(3) + config(2);
                Z = Z * config(4);
                surf(X, Y, Z, 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'none');
            end
            
            % Set view properties
            grid on;
            axis equal;
            
            % Set axis limits
            xlim([0 obj.dimensions(1)]);
            ylim([0 obj.dimensions(2)]);
            zlim([0 obj.dimensions(3)]);
            
            % Add labels
            xlabel('X (meters)');
            ylabel('Y (meters)');
            zlabel('Z (meters)');
        end

        function [X,Y,Z] = createBuildingBox(obj, pos, dims)
            % Create vertices for a 3D box
            x = pos(1) + [0 dims(1) dims(1) 0 0 dims(1) dims(1) 0];
            y = pos(2) + [0 0 dims(2) dims(2) 0 0 dims(2) dims(2)];
            z = pos(3) + [0 0 0 0 dims(3) dims(3) dims(3) dims(3)];
            
            X = reshape(x, 1, 8);
            Y = reshape(y, 1, 8);
            Z = reshape(z, 1, 8);
        end
    end
end