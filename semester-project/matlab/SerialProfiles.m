clear; clc; close all;

% Constants
T_b0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M_0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
Blist = [0 0 1 0 0.033 0;
         0 -1 0 -0.5076 0 0;
         0 -1 0 -0.3526 0 0;
         0 -1 0 -0.2176 0 0;
         0 0 1 0 0 0]';

% Initial guess
thetalist0 = [1.5; 2.5; 3; 1.25; 2.5];

% Tolerances
eomg = 0.01;
ev = 0.001;

% Circular path parameters
r = 0.1;  % 10 cm radius
zc = 0.27;  % center Z
yc = 0;     % center Y
num_pts = 100;
theta = linspace(0, 2*pi, num_pts);

% Storage for joint angles and errors
theta_mat = zeros(5, num_pts);
Y = zeros(1, num_pts);
Z = zeros(1, num_pts);

pos_errors = zeros(1,num_pts);
orient_errors = zeros(1,num_pts);

for i = 1:num_pts
    % Generate circular trajectory in Y-Z
    y = yc + r * cos(theta(i));
    z = zc + r * sin(theta(i));
    T_sd = T_b0 * M_0e;
    T_sd(2,4) = y;
    T_sd(3,4) = z;

    % Compute IK
    [theta_sol, success] = IKinBody(Blist, M_0e, T_sd, thetalist0, eomg, ev);
    if success
        thetalist0 = theta_sol;
    else
        warning('IK failed at step %d — using previous joint angles', i);
        theta_sol = thetalist0;  % fallback to last known good
    end

    % Store joint angles
    theta_mat(:, i) = theta_sol;
    Y(i) = y * 1000;  % mm
    Z(i) = z * 1000;  % mm

    % Verification: FK from IK solution
    T_fk = FKinBody(M_0e, Blist, theta_sol);

    % Position error
    pos_err = norm(T_fk(1:3,4) - T_sd(1:3,4));
    pos_errors(i) = pos_err;

    % Orientation error (angle between rotation matrices)
    R_fk = T_fk(1:3,1:3);
    R_sd = T_sd(1:3,1:3);
    R_err = R_fk' * R_sd;
    angle_err = acos((trace(R_err) - 1)/2);
    orient_errors(i) = abs(angle_err);
end

% Report max errors
fprintf('Max position error: %.5f m\n', max(pos_errors));
fprintf('Max orientation error: %.5f rad\n', max(orient_errors));

% Plot joint angles (smooth lines)
figure;
for j = 1:5
    subplot(3, 2, j);
    plot(1:num_pts, theta_mat(j,:), 'LineWidth', 1.5);
    xlabel('Path index');
    ylabel('Joint angle (rad)');
    title(['Joint trajectory \theta_', num2str(j)]);
    grid on;
end

% Cartesian trajectory in Y-Z plane
subplot(3, 2, 6);
plot(Y, Z, 'b-', 'LineWidth', 1.5);
xlabel('Y (mm)');
ylabel('Z (mm)');
title('Cartesian trajectory (position)');
axis equal;
grid on;

% Set background to white
% Set background to white
set(gcf, 'Color', 'w');

% Workaround for sgtitle on older MATLAB versions
ha = axes('Visible','off'); 
ha.Title.Visible='on';
% title(ha, 'KUKA YouBot Joint Angles and Cartesian Path');
% Assume uniform time step (e.g., 1 unit per point)
dt = 1;

% Compute joint velocities (finite differences)
theta_vel = diff(theta_mat,1,2) / dt;

% For plotting, velocity vector has length num_pts-1, so adjust time axis
t_vel = 1:(num_pts-1);

% Plot joint velocities
figure;
for j = 1:5
    subplot(3, 2, j);
    plot(t_vel, theta_vel(j,:), 'LineWidth', 1.5);
    xlabel('Path index', 'FontSize', 7);
    ylabel('Joint velocity (rad/unit time)', 'FontSize', 7);
    title(['Velocity profile \theta_', num2str(j)], 'FontSize', 10);
    grid on;
end

% Set background to white
set(gcf, 'Color', 'w');

% Add a supertitle (for older MATLAB compatibility)
ha = axes('Visible','off'); 
ha.Title.Visible='on';
% title(ha, 'KUKA YouBot Joint Velocity Profiles');
% Save joint angles to CSV file
% Add four rows of zeros to the bottom of theta_mat
% Add 4 rows of zeros to the bottom (i.e., extra joints with 0 values)
theta_mat_extended = [zeros(3, num_pts);theta_mat; zeros(5, num_pts)];
csvwrite('youbot_joint_angles.csv',theta_mat_extended');  % Each row is a time step

