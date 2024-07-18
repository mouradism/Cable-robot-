function [lj, XL] = configr()
    % Function to configure the cable lengths and coordinates

    global l1 l2 l3 La Lb

    % Define the lengths matrix
    lj = [1 1 1; 1 1 1; 1 1 0; 1 1 0] * [l1; l2; l3];

    % Check if any length exceeds the combined length of l1, l2, and l3
    if any(lj > (l1 + l2 + l3))
        error('One or more lengths exceed the total combined length.');
    end

    % Define the XL matrix
    XL = [-Lb/2, -Lb/2, +Lb/2, +Lb/2;
          -La,    0,     0,   -La];

end
