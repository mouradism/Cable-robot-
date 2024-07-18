function [X, Jk, dt_Jk, S_] = cables(lj, XL, q, dq)
    % Function to calculate the cable properties
    
    % Predefined lengths
    global l1 l2 l3
    
    % Validate input lengths
    if any(lj > l1 + l2 + l3)
        error('One or more lengths exceed the total combined length.');
    end

    % Helper function to distribute lengths
    function [l1, l2, l3] = distribute_length(l,l1, l2)
        if l <= l1
            l1 = l; l2 = 0; l3 = 0;
        elseif l <= l1 + l2
            l2 = l - l1; l3 = 0;
        else
            l3 = l - (l1 + l2);
        end
    end

    % Distribute lengths
    [l11, l12, l13] = distribute_length(lj(1),l1, l2);
    [l21, l22, l23] = distribute_length(lj(2),l1, l2);
    [l31, l32, l33] = distribute_length(lj(3),l1, l2);
    [l41, l42, l43] = distribute_length(lj(4),l1, l2);

    % Calculate Jacobians and other values
    [Jk1, dt_Jk1, X1] = jak(l11, l12, l13, q, dq);
    [Jk2, dt_Jk2, X2] = jak(l21, l22, l23, q, dq);
    [Jk3, dt_Jk3, X3] = jak(l31, l32, l33, q, dq);
    [Jk4, dt_Jk4, X4] = jak(l41, l42, l43, q, dq);

    % Calculate angles
    TETA = @(x1, x2, X) atan2(x2 - X(2), x1 - X(1));

    % Calculate S matrix
    S = zeros(2, 4);
    for i = 1:4
        S(:, i) = -[cos(TETA(XL(1, i), XL(2, i), eval(['X', num2str(i)]))); ...
                    sin(TETA(XL(1, i), XL(2, i), eval(['X', num2str(i)])))];
    end

    % Outputs
    Jk = [Jk1; Jk2; Jk3; Jk4];
    dt_Jk = [dt_Jk1; dt_Jk2; dt_Jk3; dt_Jk4];
    X = [X1; X2; X3; X4];

    % Construct S_ matrix
    S_ = blkdiag(S(:,1)', S(:,2)', S(:,3)', S(:,4)')';

end
