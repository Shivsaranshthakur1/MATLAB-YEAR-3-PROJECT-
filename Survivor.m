classdef Survivor < handle
    properties
        ID              % Unique identifier
        Position        % [x, y, z] coordinates
        Priority        % 1 (High), 2 (Medium), 3 (Low)
        Status          % 'UNDETECTED', 'IN_PROGRESS', 'DETECTED', 'RESCUED'
        Color          % RGB color based on priority
        AssignedVehicle % ID of vehicle currently assigned to this survivor
    end
    
    methods
        function obj = Survivor(id, position, priority)
            obj.ID = id;
            obj.Position = position;
            obj.Priority = priority;
            obj.Status = 'UNDETECTED';  % Use char array instead of string
            obj.AssignedVehicle = [];   % Initialize as empty
            
            % Set color based on priority
            switch priority   % Changed from 'prior' to 'priority'
                case 1 % High priority
                    obj.Color = [1 0 0];     % Red
                case 2 % Medium priority
                    obj.Color = [1 0.5 0];   % Orange
                case 3 % Low priority
                    obj.Color = [1 1 0];     % Yellow
            end
        end
    end
end