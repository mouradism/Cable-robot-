function teta = TETA(x, y, X)
    % TETA calculates the angle theta using the arctangent function.
    %
    % Parameters:
    %   x - x-coordinate of the point
    %   y - y-coordinate of the point
    %   X - 2-element vector containing the coordinates of the reference point [X1, X2]
    %
    % Returns:
    %   teta - the calculated angle in radians, adjusted by subtracting pi

    % Calculate the angle using the arctangent function and adjust by subtracting pi
    teta = atan2(y - X(2), x - X(1)) - pi;
end
