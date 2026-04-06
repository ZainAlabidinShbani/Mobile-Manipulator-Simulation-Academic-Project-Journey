function [log, metrics] = simulate_one_case_jointLQR(P, Ts, p_start, p_pick, p_place, z_lift, K, T_end)
dt = P.dt;

% Initial
q  = ik_geom_Zlink(p_start,P,'down');
dq = zeros(3,1);

N = floor(T_end/dt)+1;

% Logs
log.t    = zeros(1,N);
log.p    = zeros(3,N);
log.pd   = zeros(3,N);
log.e    = zeros(3,N);
log.tau  = zeros(3,N);
log.seg  = zeros(1,N);
log.pay  = zeros(1,N);

log.q    = zeros(3,N);     % actual
log.dq   = zeros(3,N);

log.qd   = zeros(3,N);     % desired (from IK)
log.dqd  = zeros(3,N);
log.qerr = zeros(3,N);     % qd - q
log.sig  = zeros(1,N);     % sigma_min(J)

% For numerical derivatives of qd
qd_prev  = ik_geom_Zlink(p_start,P,'down');
dqd_prev = zeros(3,1);

for k = 1:N
    t = (k-1)*dt;

    [pd, dpd, ddpd, seg_id] = pickplace_cubic(t,Ts,p_start,p_pick,p_place,z_lift);
    payload_on = (seg_id >= 2) && (seg_id <= 4);

    % Actual kinematics (for task-space error log)
    p  = fk_pos_Zlink(q,P);
    e  = pd - p;

    % Desired joint trajectory from IK (numerical dqd, qdd_d)
    qd  = ik_geom_Zlink(pd,P,'down');
    dqd = (qd - qd_prev)/dt;
    qdd_d = (dqd - dqd_prev)/dt;

    qd_prev  = qd;
    dqd_prev = dqd;

    % Joint-space LQR (x = [q_err; dq_err], u = qdd_corr)
    q_err  = q - qd;
    dq_err = dq - dqd;
    xq = [q_err; dq_err];

    qdd_cmd = qdd_d - K*xq;   % computed-torque style

    % Inverse dynamics with payload
    [M,C,G] = dyn_MCG_payload(q,dq,P,payload_on);
    tau = M*qdd_cmd + C*dq + G;

    % Forward dynamics
    qdd = M \ (tau - C*dq - G);
    dq  = dq + qdd*dt;
    q   = q  + dq*dt;

    % Guard
    if any(~isfinite(q)) || any(~isfinite(dq)) || any(~isfinite(tau))
        warning('NaN/Inf detected at t=%.4f. Stopping.', t);
        log = trim_logs(log,k-1);
        break;
    end

    % Manipulability: sigma_min(J)
    J = jacobian_pos_Zlink(q,P);
    if any(~isfinite(J(:)))
        sigmin = NaN;
    else
        s = svd(J);
        sigmin = s(end);
    end

    % Log
    log.t(k)     = t;
    log.p(:,k)   = p;
    log.pd(:,k)  = pd;
    log.e(:,k)   = e;
    log.tau(:,k) = tau;
    log.seg(k)   = seg_id;
    log.pay(k)   = payload_on;

    log.q(:,k)   = q;
    log.dq(:,k)  = dq;

    log.qd(:,k)  = qd;
    log.dqd(:,k) = dqd;
    log.qerr(:,k)= (qd - q);     % desired - actual
    log.sig(k)   = sigmin;
end

% Metrics (norm-based)
eN   = vecnorm(log.e.',2,2);
tauN = vecnorm(log.tau.',2,2);

metrics.e_norm_rms   = sqrt(mean(eN.^2));
metrics.e_norm_max   = max(eN);
metrics.tau_norm_rms = sqrt(mean(tauN.^2));
metrics.tau_norm_max = max(tauN);
end
