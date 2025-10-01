clc; clear;

% Initial configs (example)
config_i = [-0.5 -0.5 0.2 -0.1 0.1 -3*pi/4 0.1 0.1 0 0 0 0 0]';

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

% Make sure TrajectoryGenerator returns RefTraj and gripperstate
[RefTraj, gripperstate] = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k);

dt = 0.01;
N = size(RefTraj,3);

% Preallocate Vd as 6 x 1 x (N-1)
Vd = zeros(6,1,N-1);

for i = 1:N-1
    relativeTransform = TransInv(RefTraj(:,:,i)) * RefTraj(:,:,i+1);
    Tlog = MatrixLog6(relativeTransform);   % 4x4 matrix logarithm
    Vd(:,:,i) = (1/dt) * se3ToVec(Tlog);    % convert to 6x1 twist vector
end

time = dt*(0:N-2);
figure; % create new figure window
set(gcf, 'Color', [1 1 1]);  % white figure background

hold on; axis equal; grid on;
set(gca, 'Color', [1 1 1]);  % white axes background


subplot(2,1,1);
plot(time, squeeze(Vd(4:6,:,:))', 'LineWidth', 1.5); hold on; % vx, vy, vz
legend('vx', 'vy', 'vz');
xlabel('Time (s)');
ylabel('Linear Velocity (m/s)');
title('End-Effector Linear Velocity');
grid on;

subplot(2,1,2);
plot(time, squeeze(Vd(1:3,:,:))', 'LineWidth', 1.5);
legend('wx', 'wy', 'wz');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('End-Effector Angular Velocity');
grid on;

% --- Helper function ---
function V = se3ToVec(se3mat)
    omega_mat = se3mat(1:3,1:3);
    v = se3mat(1:3,4);

    % Extract skew-symmetric omega vector
    omega = [omega_mat(3,2); omega_mat(1,3); omega_mat(2,1)];

    V = [omega; v];
end
