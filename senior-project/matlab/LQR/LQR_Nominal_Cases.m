%% LQR_JOINTSPACE_3R_Zlink_INTERACTIVE_MULTI_ANIM_WITH_JOINTS_MANIP.m
% Joint-space LQR + Computed Torque (inverse dynamics)
% Multi-case interactive Q,R input + per-case animation GIF + metrics + overlays
% + Per-case plots: q actual vs desired, joint errors, manipulability sigma_min(J)

clear; clc; close all;

%% ============================ GLOBAL GRAPHICS SETTINGS ============================
set(0,'DefaultFigureColor','w');
set(0,'DefaultAxesColor','w');
set(0,'DefaultAxesXColor','k');
set(0,'DefaultAxesYColor','k');
set(0,'DefaultAxesZColor','k');
set(0,'DefaultTextColor','k');

%% ============================ 1) PARAMETERS ============================
P.L1 = 0.5; P.L2 = 0.5; P.d1 = 0.20; P.g = 9.81;

P.m1  = 2.5;
P.m2  = 2.0;
P.mp  = 0.5;
P.lc1 = P.L1/2;
P.lc2 = P.L2/2;
P.I1  = (1/12)*P.m1*P.L1^2;
P.J1  = 0.15;
P.b   = [0.03;0.04;0.02];

dt = 0.002; P.dt = dt;

%% ============================ 2) PICK & PLACE TASK ============================
p_start = [0.55;  0.00; 0.25];
p_pick  = [0.45;  0.18; 0.20];
p_place = [0.40; -0.20; 0.20];
z_lift  = 0.40;

Ts    = [2.0 1.0 2.5 1.0 2.0];
T_end = sum(Ts);

%% ============================ 3) ANIMATION SETTINGS ============================
stepAnim = 10;
gifDelay = 0.03;
cube_size = 0.04;

%% ============================ 4) INTERACTIVE MULTI-CASE INPUT ============================
fprintf('\n==================== JOINT-SPACE LQR (INTERACTIVE) ====================\n');
nCases = input('How many Q/R cases do you want to run? (e.g., 3) = ');
if isempty(nCases) || nCases < 1, nCases = 3; end

% Defaults (good starting points)
def(1).name = 'Case A (Conservative)';
def(1).Qq = 120; def(1).Qdq = 10; def(1).R = 2.5;

def(2).name = 'Case B (Balanced)';
def(2).Qq = 260; def(2).Qdq = 25; def(2).R = 1.2;

def(3).name = 'Case C (Aggressive)';
def(3).Qq = 420; def(3).Qdq = 45; def(3).R = 0.8;

cases = struct([]);
for i = 1:nCases
    if i <= numel(def)
        dname = def(i).name; dQq = def(i).Qq; dQdq = def(i).Qdq; dR = def(i).R;
    else
        dname = sprintf('Case %d', i); dQq = 260; dQdq = 25; dR = 1.2;
    end

    fprintf('\n--- Configure Case %d ---\n', i);
    name_i = input(sprintf('Case name (Enter for "%s"): ', dname),'s');
    if isempty(name_i), name_i = dname; end

    Qq_i  = input(sprintf('Qq scalar (Enter for %.1f): ', dQq));   if isempty(Qq_i),  Qq_i  = dQq;  end
    Qdq_i = input(sprintf('Qdq scalar (Enter for %.1f): ', dQdq)); if isempty(Qdq_i), Qdq_i = dQdq; end
    R_i   = input(sprintf('R scalar (Enter for %.2f): ', dR));     if isempty(R_i),   R_i   = dR;   end

    cases(i).name = name_i;
    cases(i).Qq   = Qq_i;
    cases(i).Qdq  = Qdq_i;
    cases(i).R    = R_i;
end

%% ============================ 5) RUN ALL CASES ============================
results = struct([]);

for i = 1:nCases
    % Joint-space error model: x = [q_err; dq_err], xdot = A x + B u, u = qdd_corr
    A = [ zeros(3) eye(3);
          zeros(3) zeros(3) ];
    B = [ zeros(3);
          eye(3) ];

    Q = blkdiag(cases(i).Qq*eye(3), cases(i).Qdq*eye(3));
    R = cases(i).R * eye(3);

    K = lqr(A,B,Q,R);

    fprintf('\n========== RUNNING: %s ==========\n', cases(i).name);
    fprintf('Qq=%.1f, Qdq=%.1f, R=%.3f\n', cases(i).Qq, cases(i).Qdq, cases(i).R);

    [log, metrics] = simulate_one_case_jointLQR(P, Ts, p_start, p_pick, p_place, z_lift, ...
                                                K, T_end);

    results(i).name    = cases(i).name;
    results(i).Qq      = cases(i).Qq;
    results(i).Qdq     = cases(i).Qdq;
    results(i).R       = cases(i).R;
    results(i).K       = K;
    results(i).log     = log;
    results(i).metrics = metrics;

    % Per-case animation + GIF
    gifFile = make_safe_filename(['anim_jointLQR_', cases(i).name, '.gif']);
    animate_case_to_gif(log, P, p_pick, p_place, cube_size, stepAnim, gifDelay, gifFile, cases(i).name);
    fprintf('Saved GIF: %s\n', gifFile);

    % Per-case analysis plots (joint profiles + errors + manipulability)
    plot_joint_profiles_and_manip(log, cases(i).name);
end

%% ============================ 6) OVERLAY PLOTS ============================
% Cartesian error norm overlay
figure('Color','w'); hold on; grid on;
for i=1:nCases
    e = results(i).log.e.';          % Nx3
    en = vecnorm(e,2,2);
    plot(results(i).log.t, en, 'LineWidth',1.4);
end
xlabel('Time (s)'); ylabel('||e|| (m)');
legend({results.name}, 'Location','best');
title('Cartesian Error Norm ||e(t)|| (Joint-Space LQR, Different Q/R)');

% Torque norm overlay
figure('Color','w'); hold on; grid on;
for i=1:nCases
    tau = results(i).log.tau.';       % Nx3
    taun = vecnorm(tau,2,2);
    plot(results(i).log.t, taun, 'LineWidth',1.4);
end
xlabel('Time (s)'); ylabel('||\tau|| (N·m)');
legend({results.name}, 'Location','best');
title('Torque Norm ||\tau(t)|| (Joint-Space LQR, Different Q/R)');

%% ============================ 7) METRICS TABLE ============================
caseNames = strings(nCases,1);
eRMS = zeros(nCases,1); eMAX = zeros(nCases,1);
tRMS = zeros(nCases,1); tMAX = zeros(nCases,1);

for i=1:nCases
    caseNames(i) = results(i).name;
    eRMS(i) = results(i).metrics.e_norm_rms;
    eMAX(i) = results(i).metrics.e_norm_max;
    tRMS(i) = results(i).metrics.tau_norm_rms;
    tMAX(i) = results(i).metrics.tau_norm_max;
end

metrics_table = table(caseNames, eRMS, eMAX, tRMS, tMAX, ...
    'VariableNames', {'Case','e_norm_RMS','e_norm_MAX','tau_norm_RMS','tau_norm_MAX'});
disp('==================== SUMMARY METRICS TABLE ====================');
disp(metrics_table);

%% ============================ LOCAL FUNCTIONS ============================





















