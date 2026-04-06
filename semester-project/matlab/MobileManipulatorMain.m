%% === Mobile Manipulator Control with Error Plotting ===
clc; clear; close all;

%% === Initial Configuration ===
initial_state_vector = [-0.5; -0.5; 0.2; -0.1; 0.1; -3*pi/4; 0.1; 0.1; 0; 0; 0; 0; 0];

config_ref = [0 0 1 0;
              0 1 0 0;
             -1 0 0 0.5;
              0 0 0 1];

Tse_initial = config_ref;

Tsc_initial = [1 0 0 1;
               0 1 0 0;
               0 0 1 0.025;
               0 0 0 1];

Tsc_goal = [0 1 0 0;
           -1 0 0 -1;
            0 0 1 0.025;
            0 0 0 1];

theta = pi/2;
Tce_grasp = [cos(theta) 0 sin(theta) 0;
             0          1 0          0;
            -sin(theta) 0 cos(theta) 0;
             0          0 0          1];

Tce_standoff = Tce_grasp;
Tce_standoff(3,4) = 0.2;

%% === Robot Parameters ===
dt = 0.01;
r = 0.0475;
l = 0.47 / 2;
w = 0.3 / 2;
MAX_WHEEL_VEL = 35;

H_p = r/4 * [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w);
                  1,       1,       1,        1;
                 -1,       1,      -1,        1];

T_b0 = [1 0 0 0.1662;
        0 1 0 0;
        0 0 1 0.0026;
        0 0 0 1];

M_0e = [1 0 0 0.033;
        0 1 0 0;
        0 0 1 0.6546;
        0 0 0 1];

Blist = [ 0  0  1     0     0.033   0;
          0 -1  0  -0.5076  0       0;
          0 -1  0  -0.3526  0       0;
          0 -1  0  -0.2176  0       0;
          0  0  1     0     0       0]';

%% === Trajectory Generation ===
[RefTraj, gripperstate] = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, 1);
N = size(RefTraj, 3);

Vd = ComputeDesiredTwist(RefTraj, dt, N);

%% === State Initialization ===
x = initial_state_vector(4:8);
phi = initial_state_vector(1);
base_x = initial_state_vector(2);
base_y = initial_state_vector(3);
pose = [phi; base_x; base_y];
w_current = initial_state_vector(9:12);

u_joint_des_all = zeros(N-1, 5);
u_wheel_des_all = zeros(N-1, 4);

theta_min = -pi/2 * ones(5,1);
theta_max =  pi/2 * ones(5,1);

%% === Control Loop ===
for i = 1:N-1
    [J_e, T_se] = ComputeJacobian(phi, base_x, base_y, x, T_b0, M_0e, Blist, H_p);
    des = pinv(J_e) * Vd(:,:,i);

    u_joint_des_all(i,:) = des(5:9)';
    u_wheel_des_all(i,:) = SaturateWheelVel(des(1:4), MAX_WHEEL_VEL)';

    x = x + des(5:9) * dt;
    x = max(min(x, theta_max), theta_min);
end

%% === Control & Export ===
theta_trajectories = ArmPolePlacementandObserver(u_joint_des_all', x, dt);
V_des_all = wheelsToBaseTwistPath(u_wheel_des_all');
x_log = BaseControl(V_des_all, dt, (N-1)*dt);

ExportTrajectory(x_log, theta_trajectories, N);
