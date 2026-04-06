clc; clear;

%% Physical Constants
m = 20;                  % Mass of the base in kg
Iz = 0.744;              % Moment of inertia in kg.m^2
r = 0.0475;              % Wheel radius in meters
l = 0.29;                % Half-length of base
w = 0.188;               % Half-width of base
lw = l + w;              % Sum for Jacobian term

%% Inertia Matrix (Mb)
Mb = diag([m, m, Iz]);

%% Jacobian Matrix (Jb) for Mecanum wheels
Jb = (1/r) * [
     1, -1, -lw;
     1,  1,  lw;
     1,  1, -lw;
     1, -1,  lw
];

%% Compute B matrix
B2 = inv(Mb) * Jb';       % 3x4 matrix

%% State-space matrices
% State vector: x = [x; y; theta; vx; vy; omega_z]
A = [zeros(3), eye(3);
     zeros(3), zeros(3)];

B = [zeros(3,4);
     B2];

C = eye(6);               % Full-state output
D = zeros(6,4);

%% Define desired poles for controller and observer
controller_poles = [-2, -2.5, -3, -3.5, -4, -4.5];   % Stable, fast
observer_poles = controller_poles * 5;              % Faster than controller

%% Design Gain Matrix K (Pole Placement)
K = place(A, B, controller_poles);

%% Design Observer Gain Matrix L
L = place(A', C', observer_poles)';

%% Initial Conditions
x_true = zeros(6,1);          % True initial state
x_hat = zeros(6,1);           % Estimated state

dt = 0.01;
T = 20;                      % Longer to see the circle
steps = T/dt;

log = zeros(steps, 6);        % Log for true states
wheel_vel_log = zeros(steps, 4);  % Log for wheel velocities
wheel_angle_log = zeros(steps, 4); % Log for wheel angles (integrated)

% Initialize wheel angles to zero
wheel_angles = zeros(4,1);

%% Circular Path Parameters
R = 1;               % radius (meters)
omega = 0.5;           % angular velocity (rad/s)

%% Simulation Loop
for k = 1:steps
    t = (k-1)*dt;
    
    % Desired reference moves on a circle with tangent orientation
    x_ref_pos = [R*cos(omega*t); R*sin(omega*t)];
    theta_ref = omega*t + pi/2;   % orientation tangent to circle

    % Desired state vector: [x; y; theta; vx; vy; omega_z]
    % For reference velocity, differentiate position and orientation:
    vx_ref = R*omega*sin(omega*t);
    vy_ref = R*omega*cos(omega*t);
    omega_z_ref = omega;

    x_ref = [x_ref_pos; theta_ref; vx_ref; vy_ref; omega_z_ref];

    % Control input based on estimated state
    u = -K * (x_hat - x_ref);
    
    % System dynamics
    x_dot = A * x_true + B * u;
    x_true = x_true + x_dot * dt;

    % Output (fully measured here)
    y = C * x_true;
    
    % Observer update
    y_hat = C * x_hat;
    x_hat_dot = A * x_hat + B * u + L * (y - y_hat);
    x_hat = x_hat + x_hat_dot * dt;

    % Compute wheel velocities
    v_body = x_true(4:6);                % [vx; vy; omega_z]
    wheel_vel = Jb * v_body;             % 4x1 vector
    wheel_vel_log(k,:) = wheel_vel';

    % Integrate wheel velocities to get wheel angles
    wheel_angles = wheel_angles + wheel_vel * dt;
    wheel_angle_log(k,:) = wheel_angles';

    % Log data
    log(k,:) = x_true';
end

%% Plot Results
t_vec = 0:dt:T-dt;
figure;
set(gcf, 'Color', 'w');
plot(log(:,1), log(:,2), 'LineWidth',1.5);
xlabel('x [m]');
ylabel('y [m]');
title('Mobile Base Path');
axis equal;
grid on;

figure;
set(gcf, 'Color', 'w');
plot(t_vec, log(:,1), 'r', t_vec, log(:,2), 'b', t_vec, log(:,3), 'g');
xlabel('Time [s]');
ylabel('Position [m] / Orientation [rad]');
legend('x', 'y', '\theta');
title('Mobile Base State Trajectories');
grid on;

figure;
set(gcf, 'Color', 'w');
plot(t_vec, wheel_vel_log(:,1), 'r', t_vec, wheel_vel_log(:,2), 'g', ...
     t_vec, wheel_vel_log(:,3), 'b', t_vec, wheel_vel_log(:,4), 'k');
xlabel('Time [s]');
ylabel('Wheel Angular Velocities [rad/s]');
legend('Wheel 1', 'Wheel 2', 'Wheel 3', 'Wheel 4');
title('Individual Wheel Velocities');
grid on;

figure;
set(gcf, 'Color', 'w');
plot(t_vec, wheel_angle_log(:,1), 'r', t_vec, wheel_angle_log(:,2), 'g', ...
     t_vec, wheel_angle_log(:,3), 'b', t_vec, wheel_angle_log(:,4), 'k');
xlabel('Time [s]');
ylabel('Wheel Angles [rad]');
legend('Wheel 1', 'Wheel 2', 'Wheel 3', 'Wheel 4');
title('Individual Wheel Angles');
grid on;

%% Prepare data to write
data_to_write = zeros(steps, 13);

% x, y, phi
data_to_write(:,1) = log(:,3);
data_to_write(:,2) = log(:,1);
data_to_write(:,3) = log(:,2);

% columns 4 to 8 are zeros already

% wheel angles w1,w2,w3,w4 (not velocities)
data_to_write(:,9:12) = wheel_angle_log;

% column 13 zero

%% Write CSV file
csvwrite('mobile_base_data.csv', data_to_write);

disp('CSV file "mobile_base_data.csv" saved.');