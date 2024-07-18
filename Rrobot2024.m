%% Simulation Robot 21 05 2015
% For Controller
clear all;
clf;

% Global variables
global F3_o F2_o t_ E_ TT FF g J1 J2 J3 m1 m2 m3 l1 l2 l3 La Lb r ivp Xi Xf alpha lj XL

% Initialization
FF = [];
TT = [];
F3_o = [0; 0];
F2_o = [0; 0];
t_ = 0;
E_ = 0;

% Robot parameters
g = 9.8;
Lb = 2.0;
La = 2.0;
r = 0.25;

l1 = 0.5;
l2 = 0.5;
l3 = 0.3;

m1 = 6;
m2 = 4;
m3 = 1;

J1 = (m1 * l1^2) / 3;
J2 = (m2 * l2^2) / 3;
J3 = (m3 * l3^2) / 3;

alpha = -0.0 * pi;
Xi = [-0.5; -0.40];
Xf = Xi + [0.40; -0.40];

% Link lengths
lj = [1 1 1; 0.5 0 0; 1 1 0; 1 0.5 0] * [l1; l2; l3];
if any(lj > (l1 + l2 + l3))
    return;
end

XL = [Lb / 2, -Lb / 2, Lb / 2, -Lb / 2; 0, 0, 0, 0];

% Inverse Kinematics
[q] = invKIN(Xi + l3 * [cos(alpha); sin(alpha)], Xi);

% Initial conditions
dq1 = 0; dq2 = 0; dq3 = 0;
ivp = [q(1); q(2); q(3); dq1; dq2; dq3];

% ODE solver settings
duration = 2.0;
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-3, 'MaxStep', 1e-3);
sol = ode23tb(@RRode, [0 duration], ivp, options);

% Simulation settings
fps = 60;
nframes = duration * fps;
t = linspace(0, duration, nframes);
y = deval(sol, t);

% Extract joint angles and velocities
phi1 = y(1, :)';
dtphi1 = y(4, :)';
phi2 = y(2, :)';
dtphi2 = y(5, :)';
phi3 = y(3, :)';
dtphi3 = y(6, :)';

% Plot joint angles and velocities
figure(4);
subplot(1, 4, 1);
plot(t, phi1, 'r');
hold on;
plot(t, phi2, 'g');
plot(t, phi3);
grid;

subplot(1, 4, 3);
plot(t, dtphi1, 'r');
hold on;
plot(t, dtphi2, 'g');
plot(t, dtphi3);
grid;

subplot(1, 4, 4);
plot(TT(1, :)', TT(2:end-4, :)');
grid;

subplot(1, 4, 2);
plot(TT(1, :)', TT(end-3:end, :)');
grid;

% Determine link segments
[l11, l12, l13] = calculateLinkLengths(lj(1), l1, l2, l3);
[l21, l22, l23] = calculateLinkLengths(lj(2), l1, l2, l3);
[l31, l32, l33] = calculateLinkLengths(lj(3), l1, l2, l3);
[l41, l42, l43] = calculateLinkLengths(lj(4), l1, l2, l3);

% Animation
for i = 1:length(phi1)
    figure(5);
    hold on;
    H = plot(0, 0, 'MarkerSize', 15, 'Marker', '.', 'LineWidth', 3);
    H1 = plot(0, 0, 'r', 'MarkerSize', 10, 'Marker', '.', 'LineWidth', 1);
    range = 1.5 * (l1 + l2 + l3);
    axis([-0.7 * range, 0.7 * range, -1 * range, 0.4 * range]);
    axis square;
    set(gca, 'nextplot', 'replace');

    if ishandle(H)
        Xcoord = [0, l1 * cos(phi1(i)), l1 * cos(phi1(i)) + l2 * cos(phi1(i) + phi2(i)), ...
                  l1 * cos(phi1(i)) + l2 * cos(phi1(i) + phi2(i)) + l3 * cos(phi1(i) + phi2(i) + phi3(i))];
        Ycoord = [0, l1 * sin(phi1(i)), l1 * sin(phi1(i)) + l2 * sin(phi1(i) + phi2(i)), ...
                  l1 * sin(phi1(i)) + l2 * sin(phi1(i) + phi2(i)) + l3 * sin(phi1(i) + phi2(i) + phi3(i))];
        set(H, 'XData', Xcoord, 'YData', Ycoord);
        drawnow;
    end

    Xlj1 = calculateCoordinates(l11, l12, l13, XL(:, 1), phi1(i), phi2(i), phi3(i));
    Xlj2 = calculateCoordinates(l21, l22, l23, XL(:, 2), phi1(i), phi2(i), phi3(i));
    Xlj3 = calculateCoordinates(l31, l32, l33, XL(:, 3), phi1(i), phi2(i), phi3(i));
    Xlj4 = calculateCoordinates(l41, l42, l43, XL(:, 4), phi1(i), phi2(i), phi3(i));

    if ishandle(H1)
        plotCoordinates(H1, Xlj1);
        plotCoordinates(H1, Xlj2);
        plotCoordinates(H1, Xlj3);
        plotCoordinates(H1, Xlj4);
    end

    grid;
end

% Nested function to calculate link lengths
function [l1, l2, l3] = calculateLinkLengths(lj, l1_max, l2_max, l3_max)
    if lj <= l1_max
        l1 = lj; l2 = 0; l3 = 0;
    elseif lj > l1_max && lj <= l1_max + l2_max
        l1 = l1_max; l2 = lj - l1_max; l3 = 0;
    elseif lj > l1_max + l2_max && lj <= l1_max + l2_max + l3_max
        l1 = l1_max; l2 = l2_max; l3 = lj - l1_max - l2_max;
    else
        return;
    end
end

% Nested function to calculate coordinates
function coords = calculateCoordinates(l1, l2, l3, XL_col, phi1, phi2, phi3)
    coords = [l1 * cos(phi1) + l2 * cos(phi1 + phi2) + l3 * cos(phi1 + phi2 + phi3), XL_col(1);
              l1 * sin(phi1) + l2 * sin(phi1 + phi2) + l3 * sin(phi1 + phi2 + phi3), XL_col(2)];
end

% Nested function to plot coordinates
function plotCoordinates(H, coords)
    set(H, 'XData', coords(1, :), 'YData', coords(2, :));
    drawnow;
end
