function q = invKIN(X2, X3)
    global l1 l2
    
    % Calculate the cosine of the second joint angle
    cph2r = (X2(1)^2 + X2(2)^2 - l1^2 - l2^2) / (2 * l1 * l2);
    
    % Ensure the value is within the valid range for acos to handle potential numerical errors
    cph2r = min(max(cph2r, -1), 1);
    
    % Calculate the second joint angle using the two possible solutions (elbow up and elbow down)
    q2_options = [atan2(sqrt(1 - cph2r^2), cph2r), atan2(-sqrt(1 - cph2r^2), cph2r)];
    
    % Select the elbow up solution (you may want to choose the appropriate solution based on your application)
    q(2) = q2_options(1);
    
    % Calculate the first joint angle
    q(1) = atan2(X2(2) * (l1 + l2 * cph2r) - X2(1) * l2 * sin(q(2)), ...
                 X2(1) * (l1 + l2 * cph2r) + X2(2) * l2 * sin(q(2)));
             
    % Calculate the third joint angle
    q(3) = atan2(X2(2) - X3(2), X2(1) - X3(1)) - (q(1) + q(2)) - pi;
    
    % Normalize the angles to be within the range [-pi, pi]
    q = atan2(sin(q'), cos(q'));
end
