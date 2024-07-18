clc
clear all

% Global variables
global l1 l2 l3 La Lb

% Link lengths
l1 = 0.5; 
l2 = 0.5;
l3 = 0.3;  % l3 = 0.18;
La = 2;
Lb = 2;

% Initial and final positions
Xi2 = [-0.3; -0.8];
Xi3 = [-0.6; -0.8];
Xf2 = Xi2 + [0.3; 0];
Xf3 = Xi3 + [0.3; 0];

% Compute inverse kinematics
% phi = invKIN(Xi2, Xi3);
phi = invKIN(Xf2, Xf3);

phi1 = phi(1, :);
phi2 = phi(2, :);
phi3 = phi(3, :);

% Plotting loop
for i = 1:length(phi1)
    figure(5);
    
    % Plot base point
    H = plot(0, 0, 'MarkerSize', 15, 'Marker', '.', 'LineWidth', 3); hold on;
    
    % Plot links
    H1 = plot(0, 0, 'r', 'MarkerSize', 10, 'Marker', '.', 'LineWidth', 1); hold on;
    
    % Axis settings
    range = 1.5 * (l1 + l2 + l3);
    axis([-0.7 * range 0.7 * range -range range]); 
    axis square;
    set(gca, 'nextplot', 'replacechildren'); 
    
    % Check if handle exists and plot the links
    if ishandle(H)
        Xcoord = [0, l1 * cos(phi1(i)), l1 * cos(phi1(i)) + l2 * cos(phi1(i) + phi2(i)), ...
                  l1 * cos(phi1(i)) + l2 * cos(phi1(i) + phi2(i)) + l3 * cos(phi1(i) + phi2(i) + phi3(i))];
        Ycoord = [0, l1 * sin(phi1(i)), l1 * sin(phi1(i)) + l2 * sin(phi1(i) + phi2(i)), ...
                  l1 * sin(phi1(i)) + l2 * sin(phi1(i) + phi2(i)) + l3 * sin(phi1(i) + phi2(i) + phi3(i))];
        set(H, 'XData', Xcoord, 'YData', Ycoord);
        drawnow;
    end
    
    % Plot cables
    if ishandle(H1)
        cab = [Lb / 2, 0; ...
               l1 * cos(phi1(i)) + l2 * cos(phi1(i) + phi2(i)) + l3 * cos(phi1(i) + phi2(i) + phi3(i)), ...
               l1 * sin(phi1(i)) + l2 * sin(phi1(i) + phi2(i)) + l3 * sin(phi1(i) + phi2(i) + phi3(i)); ...
              -Lb / 2, 0];
        set(H1, 'XData', cab(:, 1), 'YData', cab(:, 2));
        drawnow;
        
        % Plot boundaries
        cad = [0.35, 0; -0.35, 0; -0.35, -1.1; 0.35, -1.1; 0.35, 0];
        % plot(cad(:, 1), cad(:, 2), '--b', 'LineWidth', 1);
        
        pause(0.01);
    end
    
    % Capture frame
    F(i) = getframe;
    grid on;
    pause;
end
