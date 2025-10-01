clc; clear; close all;

%% Constants for IK circular trajectory
T_b0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M_0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
Blist = [0 0 1 0 0.033 0;
         0 -1 0 -0.5076 0 0;
         0 -1 0 -0.3526 0 0;
         0 -1 0 -0.2176 0 0;
         0 0 1 0 0 0]';

thetalist0 = [1.5; 2.5; 3; 1.25; 2.5];  % Initial guess for IK
eomg = 0.01; ev = 0.001;

r = 0.1; zc = 0.27; yc = 0;
dt = 0.1; T = 10;
time = 0:dt:T; 
num_pts = length(time);
theta_circle = linspace(0, 2*pi, num_pts);

theta_mat = zeros(5, num_pts);
Y = zeros(num_pts,1);
Z = zeros(num_pts,1);

for i = 1:num_pts
    y = yc + r * cos(theta_circle(i));
    z = zc + r * sin(theta_circle(i));
    T_sd = T_b0 * M_0e; 
    T_sd(2,4) = y; 
    T_sd(3,4) = z;
    Y(i) = y;
    Z(i) = z;
    [theta_sol, success] = IKinBody(Blist, M_0e, T_sd, thetalist0, eomg, ev);
    if success
        thetalist0 = theta_sol;
    else
        warning('IK failed at step %d, using previous joint angles', i);
        theta_sol = thetalist0;
    end
    theta_mat(:, i) = theta_sol;
end

%% Compute velocity reference trajectory by numerical differentiation
dtheta_ref_traj = diff(theta_mat, 1, 2) / dt;
dtheta_ref_traj(:, end+1) = dtheta_ref_traj(:, end);  % Pad last column

%% Call your control function with pole comparison and capture output
theta_sim = ArmPolePlacementandObserver(dtheta_ref_traj, theta_mat(:,1), dt);

%% Plot the Y-Z path of the end-effector
figure;
set(gcf, 'Color', 'w'); % White background
plot(Y, Z, 'LineWidth', 2);
xlabel('Y Position [m]');
ylabel('Z Position [m]');
title('End-Effector Path in Y-Z Plane');
grid on;
axis equal;

%% Compare simulated vs IK joint angles for one joint (optional)
figure;
set(gcf, 'Color', 'w');
plot(time, theta_mat(1,:), 'k--', 'LineWidth', 2); hold on;
plot(time, theta_sim(1,:), 'r-', 'LineWidth', 1.5);
legend('IK Reference', 'Simulated');
xlabel('Time [s]');
ylabel('\theta_1 [rad]');
title('Comparison of Reference vs Simulated Joint 1');
grid on;

%% Export simulated joint trajectories to CSV
num_samples = length(time); 
csv_data = zeros(num_samples,13); 
csv_data(:,4:8) = theta_sim';  % Use simulated joint angles instead of IK
csv_filename = 'final_try.csv';
csvwrite(csv_filename, csv_data);
