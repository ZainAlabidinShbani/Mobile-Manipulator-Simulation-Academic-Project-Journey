function theta_trajectories = ArmPolePlacementandObserver(dtheta_ref_traj, theta0, dt)

    [n, N] = size(dtheta_ref_traj);
    if n ~= 5
        error('Expected 5 joints in dtheta_ref_traj');
    end
    time = dt * (0:N-1);

    %% Generate reference from velocity
    x_ref = generate_reference_from_velocity(dtheta_ref_traj, theta0, dt);
    theta_ref_traj = x_ref(1:5, :);

    %% Dynamics matrices setup
    Mlist = cat(3, ...
        [1 0 0 0.033; 0 1 0 0; 0 0 1 0.147; 0 0 0 1], ...
        [0 0 1 0.155; 0 1 0 0; -1 0 0 0; 0 0 0 1], ...
        [1 0 0 0.135; 0 1 0 0; 0 0 1 0.038; 0 0 0 1], ...
        [1 0 0 0; 0 1 0 0; 0 0 1 0.12; 0 0 0 1], ...
        [1 0 0 0; 0 1 0 0; 0 0 1 0.1; 0 0 0 1], eye(4));
    Slist = [0 0 0 0 0; 0 1 1 1 1; 1 0 0 0 0; 0 -0.033 -0.191 -0.191 -0.191; 0 0 0 0 0; 0 0.147 0.033 0.173 0.302];
    Glist = youbot_Glist();

    M = MassMatrix(theta_ref_traj(:,1), Mlist, Glist, Slist);
    A = [zeros(n), eye(n); zeros(n), zeros(n)];
    B = [zeros(n); inv(M)];
    C = eye(2*n);
    if rank(ctrb(A,B)) < size(A,1)
        warning('System is not controllable');
    end

    if rank(obsv(A,C)) < size(A,1)
        warning('System is not observable');
    end

    %% Define multiple pole sets for controller and observer to compare
    pole_sets = {
        struct('Kpoles', linspace(-5, -30, 2*n), 'Lpoles', linspace(-40, -100, 2*n), 'label', 'Medium Poles'),
        struct('Kpoles', linspace(-10, -60, 2*n), 'Lpoles', linspace(-80, -200, 2*n), 'label', 'Fast Poles'),
        struct('Kpoles', linspace(-2, -15, 2*n), 'Lpoles', linspace(-20, -60, 2*n), 'label', 'Slow Poles')

    };

    colors = lines(n);             % Colors for joints
    M_time = length(time);
    joint_names = {'\theta_1','\theta_2','\theta_3','\theta_4','\theta_5'};

    %% Run simulations for each pole set
    for idx = 1:length(pole_sets)
        K = place(A, B, pole_sets{idx}.Kpoles);
        L = place(A', C', pole_sets{idx}.Lpoles)';

        % Initialize state vectors
        x = zeros(2*n, N);
        x_hat = zeros(2*n, N);
        x(:,1) = [theta_ref_traj(:,1); zeros(n,1)];
        x_hat(:,1) = x(:,1);

        for i = 1:N-1
            e = x_hat(:, i) - x_ref(:, i);
            u = -K * e;

            xdot = A * x(:, i) + B * u;
            x(:, i+1) = x(:, i) + xdot * dt;

            y = x(:, i);
            xhat_dot = A * x_hat(:, i) + B * u + L * (y - C * x_hat(:, i));
            x_hat(:, i+1) = x_hat(:, i) + xhat_dot * dt;
        end

        % Plot results for this pole set in a new figure
        figure('Name', pole_sets{idx}.label);
        for j = 1:n
            subplot(n,1,j);
            set(gcf, 'Color', 'w'); % gcf gets the current figure handle
            plot(time(1:M_time), theta_ref_traj(j,1:M_time), 'k--', 'LineWidth', 2.5); hold on;
            plot(time(1:M_time), x(j,1:M_time), 'Color', colors(j,:), 'LineWidth', 1.5);
            ylabel(joint_names{j});
            grid on;
            if j == 1
                title(sprintf('Joint angles (Reference dashed, Actual solid)', pole_sets{idx}.label));
                legend('Reference', 'Actual');
            end
            if j == n
                xlabel('Time [s]');
            end
        end
    end

    %% Return the calculated theta (joint angle) trajectories for the last pole set
    theta_trajectories = x(1:5, :);
end
%% Helper functions below (keep or place in separate files)

function x_ref = generate_reference_from_velocity(dtheta_ref_traj, theta0, dt)
    [n_joints, N] = size(dtheta_ref_traj);
    if n_joints ~= 5
        error('Expected 5 joints in dtheta_ref_traj');
    end

    theta_ref_traj = zeros(5, N);
    theta_ref_traj(:,1) = theta0;

    for i = 2:N
        theta_ref_traj(:,i) = theta_ref_traj(:,i-1) + dtheta_ref_traj(:,i-1) * dt;
    end

    x_ref = [theta_ref_traj; dtheta_ref_traj];
end

function Glist = youbot_Glist()
    m = [1.4, 1.2, 1.0, 0.8, 0.5];
    I = {
        diag([0.01, 0.01, 0.006]),
        diag([0.008, 0.008, 0.005]),
        diag([0.006, 0.006, 0.004]),
        diag([0.004, 0.004, 0.0025]),
        diag([0.002, 0.002, 0.001])
    };
    r = {
        [0; 0; 0.05],
        [0; 0; 0.04],
        [0; 0; 0.035],
        [0; 0; 0.03],
        [0; 0; 0.025]
    };
    Glist = zeros(6,6,5);
    for i = 1:5
        rcross = VecToso3(r{i});
        Glist(:,:,i) = [I{i} + m(i)*rcross*rcross.', m(i)*rcross; -m(i)*rcross, m(i)*eye(3)];
    end
end

function M = MassMatrix(thetalist, Mlist, Glist, Slist)
    n = length(thetalist);
    M = zeros(n);
    for i = 1:n
        ddthetalist = zeros(n,1); ddthetalist(i) = 1;
        tau = InverseDynamics(thetalist, zeros(n,1), ddthetalist, [0;0;0], zeros(6,1), Mlist, Glist, Slist);
        M(:,i) = tau;
    end
end