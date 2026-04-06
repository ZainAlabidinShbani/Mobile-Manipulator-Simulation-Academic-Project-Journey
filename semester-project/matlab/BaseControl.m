function x_log = BaseControl(v_body_desired, dt,T)
% SIMULATEMOBILEBASE Simulates a mecanum-wheeled mobile base with a state observer.
%
% Inputs:
%   v_body_desired - 3×N matrix of desired body-frame velocities: [vx; vy; omega_z]
%   dt             - Time step (s)
%   T              - Total simulation time (s)
%
% Output:
%   x_log          - Nx6 matrix of true system states over time
%                    Each row: [x, y, theta, vx, vy, omega_z]

    %% Physical Constants
    m = 20;                   % Mass in kg
    Iz = 0.744;               % Moment of inertia in kg.m^2
    r = 0.0475;               % Wheel radius
    l = 0.29;                 % Half-length
    w = 0.188;                % Half-width
    lw = l + w;               % Needed for Jacobian

    %% Inertia and Jacobian
    Mb = diag([m, m, Iz]);
    Jb = (1/r) * [
         1, -1, -lw;
         1,  1,  lw;
         1,  1, -lw;
         1, -1,  lw
    ];
    B2 = inv(Mb) * Jb';  % 3x4

    %% State-space model
    A = [zeros(3), eye(3);
         zeros(3), zeros(3)];
    B = [zeros(3,4); B2];
    C = eye(6); D = zeros(6,4);

    %% Pole placement
    controller_poles = [-3, -3.5, -4, -4.5, -5, -5.5];
    observer_poles = controller_poles * 5;
    K = place(A, B, controller_poles);
    L = place(A', C', observer_poles)';

    %% Initialization
    steps = round(T/dt);
    x_true = zeros(6,1);
    x_hat = zeros(6,1);
    wheel_angles = zeros(4,1);

    x_log = zeros(steps, 7);
    wheel_angle_log = zeros(steps, 4);
    wheel_vel_log = zeros(steps, 4);

    %% Simulation loop
    for k = 1:steps
        disp(k)
        if size(v_body_desired, 2) >= k
            v_body = v_body_desired(:,k);
        else
            v_body = v_body_desired(:,end);
        end

        % Compute reference position and orientation via integration
        if k == 1
            x_ref = zeros(6,1);
        else
            x_ref(1:3) = x_ref(1:3) + x_ref(4:6)*dt;
        end
        x_ref(4:6) = v_body;

        % Control
        u = -K * (x_hat - x_ref);

        % System update
        x_dot = A * x_true + B * u;
        x_true = x_true + x_dot * dt;

        % Observer
        y = C * x_true;
        y_hat = C * x_hat;
        x_hat_dot = A * x_hat + B * u + L * (y - y_hat);
        x_hat = x_hat + x_hat_dot * dt;

        % Wheel velocities and angles
        wheel_vel = Jb * x_true(4:6);
        a = x_true(1:3);
        wheel_vel_log(k,:) = wheel_vel';
        wheel_angles = wheel_angles + wheel_vel * dt;
        wheel_angle_log(k,:) = wheel_angles';

        % Log true state
        x_log(k,:) = [a',wheel_angles'];
    end

    %% CSV Output (optional, uncomment to enable)
    data_to_write = zeros(steps, 13);
    data_to_write(:,1) = x_log(:,3);  % phi
    data_to_write(:,2) = x_log(:,1);  % x
    data_to_write(:,3) = x_log(:,2);  % y
    data_to_write(:,9:12) = wheel_angle_log;
%     csvwrite('mobile_final_try.csv', data_to_write);
end
