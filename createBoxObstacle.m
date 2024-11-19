function createBoxObstacle(map, center, boxSize)
    % Create a box obstacle in the 3D occupancy map
    %
    % Parameters:
    % map - occupancyMap3D object
    % center - [x y z] center position of box
    % boxSize - [length width height] of box

    fprintf('Creating box obstacle at [%.1f, %.1f, %.1f] with size [%.1f, %.1f, %.1f]\n', ...
        center(1), center(2), center(3), boxSize(1), boxSize(2), boxSize(3));

    % Calculate bounds with respect to map resolution
    resolution = map.Resolution;

    % Create ranges with proper resolution
    xRange = (center(1) - boxSize(1)/2):resolution:(center(1) + boxSize(1)/2);
    yRange = (center(2) - boxSize(2)/2):resolution:(center(2) + boxSize(2)/2);
    zRange = (center(3) - boxSize(3)/2):resolution:(center(3) + boxSize(3)/2);

    % Create grid points for the box
    [X, Y, Z] = meshgrid(xRange, yRange, zRange);

    % Convert to points array
    xyzObstacle = [X(:), Y(:), Z(:)];

    % Set occupancy
    fprintf('Setting occupancy for %d points\n', size(xyzObstacle, 1));
    setOccupancy(map, xyzObstacle, ones(size(xyzObstacle, 1), 1));
    fprintf('Obstacle created successfully\n');
end