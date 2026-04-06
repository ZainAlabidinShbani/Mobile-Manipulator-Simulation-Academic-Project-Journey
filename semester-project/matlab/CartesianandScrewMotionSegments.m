clc; clear;

% === [1] Setup Initial and Goal Configurations ===
Tsc_initial = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
Tsc_goal    = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];
theta = pi/2;
Tce_grasp = [cos(theta) 0 sin(theta) 0;
             0 1 0 0;
            -sin(theta) 0 cos(theta) 0;
             0 0 0 1];
Tce_standoff = Tce_grasp; 
Tce_standoff(3,4) = 0.2;
Tse_initial = [0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
k = 1;

% === [2] Generate Trajectory ===
[RefTraj, gripperstate, isCartesianSegment] = TrajectoryGeneratorPlot( ...
    Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k);

xyz = squeeze(RefTraj(1:3,4,:))';  % Nx3 path
N = size(RefTraj,3);

% === [3] Create point-by-point color mapping array ===
pts = NaN(N,3); colors = NaN(N,3);
for i = 1:N
    pts(i,:) = xyz(i,:);
    if isCartesianSegment(i)
        colors(i,:) = [0 0 1];  % blue for Cartesian
    else
        colors(i,:) = [1 0 0];  % red for Screw
    end
end

% === [4] Plot Trajectory Paths ===
figure; hold on; axis equal; grid on;

% White background
set(gca, 'Color', [1 1 1]);
set(gcf, 'Color', [1 1 1]);

xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([0 1]);
view([45 30]);
title('Trajectory Path + RGB Axes');

% Plot path in segments
for i = 1:N-1
    c = colors(i,:);
    plot3(pts(i:i+1,1), pts(i:i+1,2), pts(i:i+1,3), '-', 'Color', c, 'LineWidth', 1.5);
end

% === [5] Animate Frames with RGB Axes every 10 steps ===
scale = 0.08; % axis arrow size
for i = 1:10:N
    T = RefTraj(:,:,i);
    p = T(1:3, 4);
    R = T(1:3, 1:3);

    % Plot RGB axes from this frame
    quiver3(p(1), p(2), p(3), R(1,1)*scale, R(2,1)*scale, R(3,1)*scale, ...
        'Color', [1 0 0], 'LineWidth', 1.5, 'MaxHeadSize', 0.5);  % X - red
    quiver3(p(1), p(2), p(3), R(1,2)*scale, R(2,2)*scale, R(3,2)*scale, ...
        'Color', [0 0.8 0], 'LineWidth', 1.5, 'MaxHeadSize', 0.5); % Y - green
    quiver3(p(1), p(2), p(3), R(1,3)*scale, R(2,3)*scale, R(3,3)*scale, ...
        'Color', [0 0 1], 'LineWidth', 1.5, 'MaxHeadSize', 0.5);  % Z - blue

    pause(0.005);
end

% === [6] Print segment change points ===
segment_changes = find(diff(isCartesianSegment) ~= 0);
segment_ends = [segment_changes(:); N];
segment_starts = [1; segment_changes(:) + 1];

for idx = 1:length(segment_ends)
    seg_start = segment_starts(idx);
    seg_end = segment_ends(idx);
    if isCartesianSegment(seg_start)
        fprintf('Ended Cartesian segment at step: %d\n', seg_end);
    else
        fprintf('Ended Screw segment at step: %d\n', seg_end);
    end
end

% === [7] Proper Legend Setup ===
h_cart = plot3(NaN,NaN,NaN,'b-','LineWidth',1.5);  % dummy blue line
h_screw = plot3(NaN,NaN,NaN,'r-','LineWidth',1.5); % dummy red line

legend([h_cart h_screw], ...
       {'Cartesian Path','Screw Path','X','Y','Z'}, ...
       'Location','northeast');
