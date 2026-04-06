%% PID_3R_Zlink_YawPitchPitch_DEMO3CASES_LAG_SAFE.m
% FULL SCRIPT — 3 demo cases with actuator lag + safe limits
% Fixes:
%   (1) Prevent leaving workspace via joint limits + accel saturation on qdd_cmd
%   (2) Pole plot shown once per case (same poles for x,y,z since gains = scalar*I)

clear; clc; close all;

%% ============================ GLOBAL GRAPHICS SETTINGS ============================
set(0,'DefaultFigureColor','w');
set(0,'DefaultAxesColor','w');
set(0,'DefaultAxesXColor','k');
set(0,'DefaultAxesYColor','k');
set(0,'DefaultAxesZColor','k');
set(0,'DefaultTextColor','k');

%% ============================ 1) PARAMETERS ============================
P.L1 = 0.5;      % [m]
P.L2 = 0.5;      % [m]
P.d1 = 0.20;     % [m] vertical offset along z
P.g  = 9.81;

% Simple dynamics assumptions
P.m1  = 2.5;                 % [kg]
P.m2  = 2.0;                 % [kg]
P.mp  = 0.5;                 % [kg] payload
P.lc1 = P.L1/2;
P.lc2 = P.L2/2;
P.I1  = (1/12)*P.m1*P.L1^2;
P.J1  = 0.15;                % yaw inertia lumped
P.b   = [0.03;0.04;0.02];    % viscous friction

dt = 0.002;     % [s]
P.dt = dt;

% ---- Joint limits (prevent exiting workspace) ----
% yaw is free-ish, pitch joints limited to a reasonable range
P.qmin = [-pi;   -pi/2;   -pi];      % [rad]
P.qmax = [ pi;    pi/2;    pi/2];    % [rad]
P.dqmax = [6; 6; 6];                 % [rad/s] soft velocity cap

%% ============================ 2) PICK & PLACE TASK ============================
p_start = [0.55;  0.00; 0.25];
p_pick  = [0.45;  0.18; 0.20];
p_place = [0.40; -0.20; 0.20];
z_lift  = 0.40;

Ts    = [2.0 1.0 2.5 1.0 2.0];
Tmin  = min(Ts);

wn0 = 6 / Tmin;  % baseline

%% ============================ 3) CONTROLLER + LAG SETTINGS ============================
alphaKi = 0.06;
ei_max  = 0.15;

% Actuator / servo lag
wa = 20;         % [rad/s]
za = 0.7;        % [-]
qdd_lim = 35;    % [rad/s^2] accel saturation

%% ============================ 4) DEMO CASES ============================
CASES = struct([]);

CASES(1).name = 'A_under_FAST';
CASES(1).zeta = 0.50;
CASES(1).wn   = 2.00*wn0;

CASES(2).name = 'B_nearcrit_NOM';
CASES(2).zeta = 0.95;
CASES(2).wn   = 1.00*wn0;

CASES(3).name = 'C_over_SLOW';
CASES(3).zeta = 1.40;
CASES(3).wn   = 0.60*wn0;

% Animation settings
makeGIF  = true;
stepAnim = 6;
gifDelay = 0.03;

%% ============================ 5) POLE PLOT (ONCE per case) ============================
figure('Color','w'); hold on; grid on;
title('Closed-loop poles per case (same poles apply to x,y,z since gains = scalar \times I)');
xlabel('Real axis'); ylabel('Imag axis');
if exist('sgrid','file') == 2
    sgrid;
end

cols = lines(numel(CASES));
for i = 1:numel(CASES)
    zeta = CASES(i).zeta;
    wn   = CASES(i).wn;

    poles_PD  = roots([1, 2*zeta*wn, wn^2]);
    poles_PID = roots([1, 2*zeta*wn, wn^2, alphaKi*wn^3]);

    CASES(i).poles_PD  = poles_PD;
    CASES(i).poles_PID = poles_PID;

    plot(real(poles_PD),  imag(poles_PD),  'x', 'LineWidth', 1.8, 'Color', cols(i,:));
    plot(real(poles_PID), imag(poles_PID), 'o', 'LineWidth', 1.4, 'Color', cols(i,:));
end
legend({'PD poles','PID poles'},'Location','best');

text(0.02,0.02, ...
    'Note: Controller gains are identical across Cartesian axes, so pole locations repeat for x,y,z.', ...
    'Units','normalized','FontSize',9,'Color',[0.2 0.2 0.2]);

%% ============================ 6) RUN CASES ============================
for i = 1:numel(CASES)
    zeta = CASES(i).zeta;
    wn   = CASES(i).wn;

    % Gains
    Kp = (wn^2) * eye(3);
    Kd = (2*zeta*wn) * eye(3);
    Ki = (alphaKi*wn^3) * eye(3);

    cfg = struct();
    cfg.name   = CASES(i).name;
    cfg.zeta   = zeta;
    cfg.wn     = wn;
    cfg.Kp     = Kp;
    cfg.Kd     = Kd;
    cfg.Ki     = Ki;
    cfg.ei_max = ei_max;

    cfg.wa      = wa;
    cfg.za      = za;
    cfg.qdd_lim = qdd_lim;

    fprintf('\n==================== %s ====================\n', CASES(i).name);
    fprintf('zeta=%.2f, wn=%.3f rad/s | Kp=%.3f Kd=%.3f Ki=%.3f\n', ...
        zeta, wn, wn^2, 2*zeta*wn, alphaKi*wn^3);
    fprintf('Actuator lag: wa=%.1f rad/s, za=%.2f, qdd_lim=%.1f rad/s^2\n', wa, za, qdd_lim);

    [log, metrics] = run_case(P, cfg, Ts, p_start, p_pick, p_place, z_lift, dt);

    CASES(i).log     = log;
    CASES(i).metrics = metrics;

    fprintf('RMS||e||=%.3e  MAX||e||=%.3e | RMS||tau||=%.3e  MAX||tau||=%.3e | sigMin(J)=%.3e\n', ...
        metrics.e_norm_rms, metrics.e_norm_max, metrics.tau_norm_rms, metrics.tau_norm_max, metrics.sig_min);

    % Plots per case
    plot_case_results(CASES(i), p_start, p_pick, p_place);

    % GIF
    if makeGIF
        gifFile = sprintf('%s_z%.2f_wn%.2f.gif', CASES(i).name, zeta, wn);
        animate_case(P, log, p_pick, p_place, stepAnim, gifFile, gifDelay, CASES(i).name);
        fprintf('Saved GIF: %s\n', gifFile);
    end
end

%% ============================ 7) SUMMARY TABLE ============================
nC = numel(CASES);
name  = strings(nC,1);
zetaV = zeros(nC,1);
wnV   = zeros(nC,1);
KpV   = zeros(nC,1);
KdV   = zeros(nC,1);
KiV   = zeros(nC,1);

eRMS  = zeros(nC,1);
eMAX  = zeros(nC,1);
tRMS  = zeros(nC,1);
tMAX  = zeros(nC,1);
sigMn = zeros(nC,1);

for i=1:nC
    name(i)  = CASES(i).name;
    zetaV(i) = CASES(i).zeta;
    wnV(i)   = CASES(i).wn;

    KpV(i) = wnV(i)^2;
    KdV(i) = 2*zetaV(i)*wnV(i);
    KiV(i) = alphaKi*wnV(i)^3;

    eRMS(i)  = CASES(i).metrics.e_norm_rms;
    eMAX(i)  = CASES(i).metrics.e_norm_max;
    tRMS(i)  = CASES(i).metrics.tau_norm_rms;
    tMAX(i)  = CASES(i).metrics.tau_norm_max;
    sigMn(i) = CASES(i).metrics.sig_min;
end

summaryTable = table(name, zetaV, wnV, KpV, KdV, KiV, eRMS, eMAX, tRMS, tMAX, sigMn, ...
    'VariableNames', {'caseName','zeta','wn','Kp','Kd','Ki','eRMS','eMAX','tauRMS','tauMAX','sigMinJ'});

disp(' ');
disp('==================== SUMMARY TABLE ====================');
disp(summaryTable);

%% ============================ LOCAL FUNCTIONS ============================

function [log, metrics] = run_case(P, cfg, Ts, p_start, p_pick, p_place, z_lift, dt)

T_end = sum(Ts);
N = floor(T_end/dt)+1;

q  = ik_geom_Zlink(p_start,P,'down');
dq = zeros(3,1);
ei = zeros(3,1);

% Actuator lag states
qdd_f  = zeros(3,1);
dqdd_f = zeros(3,1);

% Logs
log.t    = zeros(1,N);
log.p    = zeros(3,N);
log.pd   = zeros(3,N);
log.e    = zeros(3,N);
log.tau  = zeros(3,N);
log.sig  = zeros(1,N);
log.seg  = zeros(1,N);
log.pay  = zeros(1,N);
log.q    = zeros(3,N);
log.qd   = zeros(3,N);

Kp = cfg.Kp; Kd = cfg.Kd; Ki = cfg.Ki;
ei_max = cfg.ei_max;

wa      = cfg.wa;
za      = cfg.za;
qdd_lim = cfg.qdd_lim;

for k = 1:N
    t = (k-1)*dt;

    [pd, dpd, ddpd, seg_id] = pickplace_cubic(t,Ts,p_start,p_pick,p_place,z_lift);
    payload_on = (seg_id >= 2) && (seg_id <= 4);

    % Kinematics
    p  = fk_pos_Zlink(q,P);
    J  = jacobian_pos_Zlink(q,P);
    dp = J*dq;

    e  = pd - p;
    de = dpd - dp;

    % Integral with clamp
    ei = ei + e*dt;
    ei = max(-ei_max, min(ei_max, ei));

    % Task-space accel command
    ddp_cmd = ddpd + Kd*de + Kp*e + Ki*ei;

    % Jdot numerical
    eps = 1e-6;
    J2 = jacobian_pos_Zlink(q + dq*eps, P);
    Jdot = (J2 - J)/eps;

    % Conditioning and adaptive DLS (simple robustness)
    s = svd(J);
    sigmin = s(end);
    lambda = 1e-4;
    if sigmin < 0.08
        lambda = 5e-3;
    elseif sigmin < 0.15
        lambda = 1e-3;
    end

    % DLS mapping
    qdd_cmd = (J'*J + lambda*eye(3)) \ (J'*(ddp_cmd - Jdot*dq));

    % --- NEW: saturate qdd_cmd BEFORE actuator lag (prevents runaway) ---
    qdd_cmd = max(-qdd_lim, min(qdd_lim, qdd_cmd));

    % Inverse dynamics torque (based on commanded accel)
    [M,C,G] = dyn_MCG_payload(q,dq,P,payload_on);
    tau = M*qdd_cmd + C*dq + G;

    % ============================================================
    % Actuator/servo lag on acceleration command (2nd order)
    ddqdd_f = wa^2*(qdd_cmd - qdd_f) - 2*za*wa*dqdd_f;
    dqdd_f  = dqdd_f + ddqdd_f*dt;
    qdd_f   = qdd_f  + dqdd_f*dt;

    % Saturation on realized accel
    qdd_f = max(-qdd_lim, min(qdd_lim, qdd_f));
    qdd   = qdd_f;
    % ============================================================

    % Integrate
    dq  = dq + qdd*dt;

    % Soft dq limit
    dq = max(-P.dqmax, min(P.dqmax, dq));

    q   = q  + dq*dt;

    % --- NEW: joint limit clamp (keeps arm in feasible workspace) ---
    q = max(P.qmin, min(P.qmax, q));

    % Desired joints for plotting (IK of desired)
    qd = ik_geom_Zlink(pd,P,'down');

    % Log
    log.t(k)    = t;
    log.p(:,k)  = p;
    log.pd(:,k) = pd;
    log.e(:,k)  = e;
    log.tau(:,k)= tau;
    log.sig(k)  = sigmin;
    log.seg(k)  = seg_id;
    log.pay(k)  = payload_on;
    log.q(:,k)  = q;
    log.qd(:,k) = qd;

    % Safety break if numerical issues
    if any(~isfinite(q)) || any(~isfinite(dq))
        warning('Numerical issue detected. Stopping simulation early.');
        log.t    = log.t(1:k);
        log.p    = log.p(:,1:k);
        log.pd   = log.pd(:,1:k);
        log.e    = log.e(:,1:k);
        log.tau  = log.tau(:,1:k);
        log.sig  = log.sig(1:k);
        log.seg  = log.seg(1:k);
        log.pay  = log.pay(1:k);
        log.q    = log.q(:,1:k);
        log.qd   = log.qd(:,1:k);
        break;
    end
end

% ===== metrics =====
e   = log.e.';        % Nx3
tau = log.tau.';      % Nx3
sig = log.sig(:);

e_norm = vecnorm(e,2,2);
tau_norm = vecnorm(tau,2,2);

metrics.e_norm_rms = sqrt(mean(e_norm.^2));
metrics.e_norm_max = max(e_norm);

metrics.tau_norm_rms = sqrt(mean(tau_norm.^2));
metrics.tau_norm_max = max(tau_norm);

metrics.sig_min  = min(sig);
metrics.sig_mean = mean(sig);

end










