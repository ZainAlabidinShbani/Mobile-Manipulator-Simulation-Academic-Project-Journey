%% FUZZY_PID_3R_Zlink_YawPitchPitch_FULL_NO_TOOLBOX_WITH_LAG.m
% 3R arm: Joint1 yaw about z, Link1 along z (d1), joints 2&3 pitch in vertical plane
% Task-space FUZZY gain-scheduled PID + inverse dynamics + payload cube
% Manual Mamdani-like inference (NO Fuzzy Logic Toolbox)
% + LAG/DELAY case (reference delay and/or feedback delay)
% + Membership Functions plots (for report)

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
P.d1 = 0.20;     % [m] FIRST LINK ALONG Z (vertical offset)
P.g  = 9.81;

% Dynamics assumptions (keep simple)
P.m1  = 2.5;                 % [kg]
P.m2  = 2.0;                 % [kg]
P.mp  = 0.5;                 % [kg] payload mass (cube)
P.lc1 = P.L1/2;
P.lc2 = P.L2/2;
P.I1  = (1/12)*P.m1*P.L1^2;
P.J1  = 0.15;
P.b   = [0.03;0.04;0.02];

dt = 0.002;
P.dt = dt;

%% ============================ 2) PICK & PLACE TASK ============================
p_start = [0.55;  0.00; 0.25];
p_pick  = [0.45;  0.18; 0.20];
p_place = [0.40; -0.20; 0.20];
z_lift  = 0.40;

Ts    = [2.0 1.0 2.5 1.0 2.0];
T_end = sum(Ts);
Tmin  = min(Ts);

%% ============================ 3) BASE "PID" GAINS (NOMINAL) ============================
zeta = 0.9;
wn = 6 / Tmin;

Kp0 = (wn^2) * eye(3);
Kd0 = (2*zeta*wn) * eye(3);
Ki0 = (0.08*wn^3) * eye(3);

ei_max = 0.10;

fprintf('\n=== Task-Space Fuzzy-PID (Gain-Scheduled, NO Toolbox) ===\n');
fprintf('Tmin = %.3f s, wn = %.3f rad/s, zeta = %.2f\n', Tmin, wn, zeta);
fprintf('Nominal PID gains are BASE gains then scaled by fuzzy factors (manual inference).\n');

%% ============================ 3.1) NORMALIZATION ============================
Ke  = 1/0.10;   % typical |e| ~ 0.10 m -> en ~ 1
Kde = 1/0.50;   % typical |de| ~ 0.50 m/s -> den ~ 1

%% ============================ 3.2) LAG / DELAY SETTINGS ============================
% Choose delay type(s):
useDelayOnReference = false;  % true: pd,dpd,ddpd are delayed (command/reference delay)
useDelayOnFeedback  = true;   % true: p,dp are delayed (sensor/measurement delay)

Tdelay = 0.05;                % [s] lag amount (e.g., 50 ms)
Nd = round(Tdelay/dt);        % delay in samples

fprintf('\n=== LAG CASE SETTINGS ===\n');
fprintf('Tdelay = %.3f s (Nd = %d samples)\n', Tdelay, Nd);
fprintf('Delay on reference: %d\n', useDelayOnReference);
fprintf('Delay on feedback : %d\n', useDelayOnFeedback);

%% ============================ 4) INITIAL CONDITIONS ============================
q  = ik_geom_Zlink(p_start,P,'down');
dq = zeros(3,1);
ei = zeros(3,1);

N = floor(T_end/dt)+1;

% Buffers for delay (initialize safely)
pd_buf   = repmat(p_start,1,max(Nd,0)+1);
dpd_buf  = zeros(3,max(Nd,0)+1);
ddpd_buf = zeros(3,max(Nd,0)+1);

p_buf  = repmat(p_start,1,max(Nd,0)+1);
dp_buf = zeros(3,max(Nd,0)+1);

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

% Fuzzy scalers logs
log.ap = zeros(3,N);
log.ai = zeros(3,N);
log.ad = zeros(3,N);

%% ============================ 5) MAIN SIMULATION ============================
for k = 1:N
    t = (k-1)*dt;

    % Desired trajectory (no delay yet)
    [pd, dpd, ddpd, seg_id] = pickplace_cubic(t,Ts,p_start,p_pick,p_place,z_lift);

    % Payload ON after pick until place (segments 2..4)
    payload_on = (seg_id >= 2) && (seg_id <= 4);

    % ---------- Update reference buffers (optional reference delay) ----------
    if Nd > 0
        pd_buf   = [pd_buf(:,2:end),   pd];
        dpd_buf  = [dpd_buf(:,2:end),  dpd];
        ddpd_buf = [ddpd_buf(:,2:end), ddpd];
    end

    if useDelayOnReference && Nd>0
        pd_use   = pd_buf(:,1);
        dpd_use  = dpd_buf(:,1);
        ddpd_use = ddpd_buf(:,1);
    else
        pd_use   = pd;
        dpd_use  = dpd;
        ddpd_use = ddpd;
    end

    % Kinematics (current, true)
    p  = fk_pos_Zlink(q,P);
    J  = jacobian_pos_Zlink(q,P);
    dp = J*dq;

    % ---------- Update feedback buffers (optional feedback delay) ----------
    if Nd > 0
        p_buf  = [p_buf(:,2:end),  p];
        dp_buf = [dp_buf(:,2:end), dp];
    end

    if useDelayOnFeedback && Nd>0
        p_use  = p_buf(:,1);
        dp_use = dp_buf(:,1);
    else
        p_use  = p;
        dp_use = dp;
    end

    % Task-space errors (computed using delayed signals if enabled)
    e  = pd_use - p_use;
    de = dpd_use - dp_use;

    % Integrator (anti-windup)
    ei = ei + e*dt;
    ei = max(-ei_max, min(ei_max, ei));

    % ===================== FUZZY PID-LIKE BLOCK =====================
    % Normalize and saturate inputs
    en  = max(-1, min(1, Ke  * e));
    den = max(-1, min(1, Kde * de));

    % Per-axis fuzzy scaling factors alpha in [0,1]
    ap = zeros(3,1); ai = zeros(3,1); ad = zeros(3,1);
    for i = 1:3
        ap(i) = evalGainFIS_manual(en(i), den(i), 'P');
        ai(i) = evalGainFIS_manual(en(i), den(i), 'I');
        ad(i) = evalGainFIS_manual(en(i), den(i), 'D');
    end

    % Effective gains (diagonal scheduling)
    Kp_eff = diag(ap) * Kp0;
    Ki_eff = diag(ai) * Ki0;
    Kd_eff = diag(ad) * Kd0;

    % Commanded Cartesian acceleration (use delayed ref if enabled)
    ddp_cmd = ddpd_use + Kd_eff*de + Kp_eff*e + Ki_eff*ei;
    % =================================================================

    % Jdot numerical
    epsJ = 1e-6;
    J2 = jacobian_pos_Zlink(q + dq*epsJ, P);
    Jdot = (J2 - J)/epsJ;

    % DLS mapping
    lambda = 1e-4;
    qdd_cmd = (J'*J + lambda*eye(3)) \ (J'*(ddp_cmd - Jdot*dq));

    % Inverse dynamics with payload
    [M,C,G] = dyn_MCG_payload(q,dq,P,payload_on);
    tau = M*qdd_cmd + C*dq + G;

    % Forward dynamics
    qdd = M \ (tau - C*dq - G);
    dq  = dq + qdd*dt;
    q   = q  + dq*dt;

    % Desired joint angles for plotting (IK on desired CURRENT ref (not delayed))
    qd = ik_geom_Zlink(pd,P,'down');

    % Conditioning
    s = svd(J);
    sigmin = s(end);

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

    log.ap(:,k) = ap;
    log.ai(:,k) = ai;
    log.ad(:,k) = ad;
end

%% ============================ 6) PLOTS ============================
figure('Color','w'); hold on; grid on; axis equal;
plot3(log.pd(1,:),log.pd(2,:),log.pd(3,:),'k--','LineWidth',1.6);
plot3(log.p(1,:), log.p(2,:), log.p(3,:), 'b','LineWidth',1.6);
plot3(p_start(1),p_start(2),p_start(3),'bo','MarkerSize',8,'MarkerFaceColor','b');
plot3(p_pick(1), p_pick(2), p_pick(3), 'go','MarkerSize',8,'MarkerFaceColor','g');
plot3(p_place(1),p_place(2),p_place(3),'mo','MarkerSize',8,'MarkerFaceColor','m');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('Desired Path','Actual Path','Start','Pick','Place','Location','best');
title('Pick-and-Place Cartesian Path (Fuzzy-PID with Lag Option)');

figure('Color','w');
subplot(3,1,1); plot(log.t,log.e(1,:),'LineWidth',1.2); grid on; ylabel('e_x (m)');
subplot(3,1,2); plot(log.t,log.e(2,:),'LineWidth',1.2); grid on; ylabel('e_y (m)');
subplot(3,1,3); plot(log.t,log.e(3,:),'LineWidth',1.2); grid on; ylabel('e_z (m)'); xlabel('Time (s)');
my_sgtitle('Task-Space Tracking Errors');

figure('Color','w');
subplot(3,1,1); plot(log.t,log.tau(1,:),'LineWidth',1.1); grid on; ylabel('\tau_1 (N.m)');
subplot(3,1,2); plot(log.t,log.tau(2,:),'LineWidth',1.1); grid on; ylabel('\tau_2 (N.m)');
subplot(3,1,3); plot(log.t,log.tau(3,:),'LineWidth',1.1); grid on; ylabel('\tau_3 (N.m)'); xlabel('Time (s)');
my_sgtitle('Joint Torques (Inverse Dynamics)');

figure('Color','w');
plot(log.t,log.sig,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('\sigma_{min}(J)'); title('Jacobian Conditioning Indicator');

figure('Color','w');
stairs(log.t,log.pay,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Payload ON (0/1)'); title('Payload Activation During Pick-and-Place');

figure('Color','w');
subplot(3,1,1); plot(log.t,log.q(1,:),'LineWidth',1.2); hold on; plot(log.t,log.qd(1,:),'--','LineWidth',1.2);
grid on; ylabel('q_1 (rad)'); legend('Actual','Desired','Location','best');
subplot(3,1,2); plot(log.t,log.q(2,:),'LineWidth',1.2); hold on; plot(log.t,log.qd(2,:),'--','LineWidth',1.2);
grid on; ylabel('q_2 (rad)'); legend('Actual','Desired','Location','best');
subplot(3,1,3); plot(log.t,log.q(3,:),'LineWidth',1.2); hold on; plot(log.t,log.qd(3,:),'--','LineWidth',1.2);
grid on; ylabel('q_3 (rad)'); xlabel('Time (s)'); legend('Actual','Desired','Location','best');
my_sgtitle('Joint Angles Tracking (Actual vs Desired)');

% Fuzzy scalers plots (report-friendly)
figure('Color','w');
subplot(3,1,1);
plot(log.t,log.ap(1,:),'LineWidth',1.2); hold on;
plot(log.t,log.ap(2,:),'LineWidth',1.2);
plot(log.t,log.ap(3,:),'LineWidth',1.2);
grid on; ylabel('\alpha_p'); legend('x','y','z'); title('Fuzzy Proportional Gain Scaler');

subplot(3,1,2);
plot(log.t,log.ad(1,:),'LineWidth',1.2); hold on;
plot(log.t,log.ad(2,:),'LineWidth',1.2);
plot(log.t,log.ad(3,:),'LineWidth',1.2);
grid on; ylabel('\alpha_d'); legend('x','y','z'); title('Fuzzy Derivative Gain Scaler');

subplot(3,1,3);
plot(log.t,log.ai(1,:),'LineWidth',1.2); hold on;
plot(log.t,log.ai(2,:),'LineWidth',1.2);
plot(log.t,log.ai(3,:),'LineWidth',1.2);
grid on; ylabel('\alpha_i'); xlabel('Time (s)'); legend('x','y','z'); title('Fuzzy Integral Gain Scaler');

%% ============================ 7) ANIMATION + PATH DRAWING ============================
figure('Color','w'); hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45,25);
title('3R Arm Animation (Link1 along Z) + Live Path');
xlim([-0.8 0.8]); ylim([-0.8 0.8]); zlim([0 0.7]);

plot3(0,0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');

h_l0 = plot3([0 0],[0 0],[0 0],'b','LineWidth',3);
h_l1 = plot3([0 0],[0 0],[0 0],'r','LineWidth',3);
h_l2 = plot3([0 0],[0 0],[0 0],'g','LineWidth',3);
h_ee = plot3(0,0,0,'ro','MarkerSize',6,'MarkerFaceColor','r');

h_path_act = plot3(NaN,NaN,NaN,'b','LineWidth',1.6);
h_path_des = plot3(NaN,NaN,NaN,'k--','LineWidth',1.2);

cube_size = 0.04;
h_cube = patch('Faces',[1 2 3 4 5 6 7 8], ...
    'Vertices',cube_vertices(p_pick,cube_size), ...
    'FaceColor',[0.2 0.8 0.3],'EdgeColor','k');

Xa=[]; Ya=[]; Za=[];
Xd=[]; Yd=[]; Zd=[];

stepAnim = 10;
gifFile = 'pick_and_place_animation.gif';
gifDelay = 0.03;
isFirstFrame = true;

for k = 1:stepAnim:length(log.t)
    qk = log.q(:,k);
    th1=qk(1); th2=qk(2); th3=qk(3);

    p0 = [0;0;0];
    p1 = [0;0;P.d1];

    r2 = P.L1*cos(th2);
    z2 = P.d1 + P.L1*sin(th2);
    p2 = [cos(th1)*r2; sin(th1)*r2; z2];

    r3 = P.L1*cos(th2) + P.L2*cos(th2+th3);
    z3 = P.d1 + P.L1*sin(th2) + P.L2*sin(th2+th3);
    p3 = [cos(th1)*r3; sin(th1)*r3; z3];

    set(h_l0,'XData',[p0(1) p1(1)],'YData',[p0(2) p1(2)],'ZData',[p0(3) p1(3)]);
    set(h_l1,'XData',[p1(1) p2(1)],'YData',[p1(2) p2(2)],'ZData',[p1(3) p2(3)]);
    set(h_l2,'XData',[p2(1) p3(1)],'YData',[p2(2) p3(2)],'ZData',[p2(3) p3(3)]);
    set(h_ee,'XData',p3(1),'YData',p3(2),'ZData',p3(3));

    if log.pay(k)==1
        set(h_cube,'Vertices',cube_vertices(p3,cube_size));
    else
        if log.seg(k) < 2
            set(h_cube,'Vertices',cube_vertices(p_pick,cube_size));
        else
            set(h_cube,'Vertices',cube_vertices(p_place,cube_size));
        end
    end

    Xa(end+1)=p3(1); Ya(end+1)=p3(2); Za(end+1)=p3(3);
    Xd(end+1)=log.pd(1,k); Yd(end+1)=log.pd(2,k); Zd(end+1)=log.pd(3,k);

    set(h_path_act,'XData',Xa,'YData',Ya,'ZData',Za);
    set(h_path_des,'XData',Xd,'YData',Yd,'ZData',Zd);

    drawnow;
    pause(0.01);

    frame = getframe(gcf);
    img = frame2im(frame);
    [imind, cm] = rgb2ind(img, 256);

    if isFirstFrame
        imwrite(imind, cm, gifFile, 'gif', 'Loopcount', inf, 'DelayTime', gifDelay);
        isFirstFrame = false;
    else
        imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
    end
end

%% ============================ 8) METRICS EXTRACTION ============================
t = log.t(:);
e = log.e.';        % Nx3
tau = log.tau.';    % Nx3
pay = log.pay(:) > 0;
sig = log.sig(:);

Ttotal = t(end) - t(1);
ss_idx = t >= (t(1) + 0.90*Ttotal);
pay_idx = pay;
nopay_idx = ~pay;

e_rms = sqrt(mean(e.^2, 1));
e_max = max(abs(e), [], 1);
e_ss_abs_mean = mean(abs(e(ss_idx, :)), 1);

e_norm = vecnorm(e,2,2);
e_norm_rms = sqrt(mean(e_norm.^2));
e_norm_max = max(e_norm);

if any(pay_idx)
    e_rms_pay = sqrt(mean(e(pay_idx,:).^2, 1));
    e_max_pay = max(abs(e(pay_idx,:)), [], 1);
    e_norm_rms_pay = sqrt(mean(e_norm(pay_idx).^2));
    e_norm_max_pay = max(e_norm(pay_idx));
else
    e_rms_pay = [NaN NaN NaN];
    e_max_pay = [NaN NaN NaN];
    e_norm_rms_pay = NaN;
    e_norm_max_pay = NaN;
end

tau_rms = sqrt(mean(tau.^2, 1));
tau_max = max(abs(tau), [], 1);

if any(pay_idx)
    tau_rms_pay = sqrt(mean(tau(pay_idx,:).^2, 1));
    tau_max_pay = max(abs(tau(pay_idx,:)), [], 1);
else
    tau_rms_pay = [NaN NaN NaN];
    tau_max_pay = [NaN NaN NaN];
end

if any(nopay_idx)
    tau_rms_nopay = sqrt(mean(tau(nopay_idx,:).^2, 1));
    tau_max_nopay = max(abs(tau(nopay_idx,:)), [], 1);
else
    tau_rms_nopay = [NaN NaN NaN];
    tau_max_nopay = [NaN NaN NaN];
end

tau_rms_inc_pct = 100*(tau_rms_pay - tau_rms_nopay) ./ max(tau_rms_nopay, 1e-12);
tau_max_inc_pct = 100*(tau_max_pay - tau_max_nopay) ./ max(tau_max_nopay, 1e-12);

sig_min = min(sig);
sig_mean = mean(sig);

fprintf('\n==================== METRICS SUMMARY (FUZZY with LAG OPTION) ====================\n');
fprintf('Cartesian Error RMS [ex ey ez] (m):      %.3e  %.3e  %.3e\n', e_rms(1), e_rms(2), e_rms(3));
fprintf('Cartesian Error MAX [ex ey ez] (m):      %.3e  %.3e  %.3e\n', e_max(1), e_max(2), e_max(3));
fprintf('Steady-state |e| mean (last 10%%) (m):    %.3e  %.3e  %.3e\n', e_ss_abs_mean(1), e_ss_abs_mean(2), e_ss_abs_mean(3));
fprintf('||e|| RMS (m):                           %.3e\n', e_norm_rms);
fprintf('||e|| MAX (m):                           %.3e\n', e_norm_max);

if any(pay_idx)
    fprintf('\n--- Payload ON window metrics ---\n');
    fprintf('Cartesian Error RMS [ex ey ez] (m):      %.3e  %.3e  %.3e\n', e_rms_pay(1), e_rms_pay(2), e_rms_pay(3));
    fprintf('Cartesian Error MAX [ex ey ez] (m):      %.3e  %.3e  %.3e\n', e_max_pay(1), e_max_pay(2), e_max_pay(3));
    fprintf('||e|| RMS (m):                           %.3e\n', e_norm_rms_pay);
    fprintf('||e|| MAX (m):                           %.3e\n', e_norm_max_pay);
end

fprintf('\nJoint Torque RMS [t1 t2 t3] (N.m):       %.3e  %.3e  %.3e\n', tau_rms(1), tau_rms(2), tau_rms(3));
fprintf('Joint Torque MAX [t1 t2 t3] (N.m):       %.3e  %.3e  %.3e\n', tau_max(1), tau_max(2), tau_max(3));

if any(pay_idx) && any(nopay_idx)
    fprintf('\nTorque increase due to payload (RMS) %%:  %.2f%%  %.2f%%  %.2f%%\n', tau_rms_inc_pct(1), tau_rms_inc_pct(2), tau_rms_inc_pct(3));
    fprintf('Torque increase due to payload (MAX) %%:  %.2f%%  %.2f%%  %.2f%%\n', tau_max_inc_pct(1), tau_max_inc_pct(2), tau_max_inc_pct(3));
end

fprintf('\nJacobian sigma_min(J):                   %.3e\n', sig_min);
fprintf('Jacobian mean sigma_min(J):              %.3e\n', sig_mean);
fprintf('===============================================================================\n');

metrics_table = table( ...
    e_rms(:), e_max(:), e_ss_abs_mean(:), e_rms_pay(:), e_max_pay(:), ...
    tau_rms(:), tau_max(:), tau_rms_pay(:), tau_max_pay(:), ...
    'VariableNames', {'e_RMS','e_MAX','e_SS_abs_mean','e_RMS_payload','e_MAX_payload', ...
                      'tau_RMS','tau_MAX','tau_RMS_payload','tau_MAX_payload'}, ...
    'RowNames', {'X','Y','Z'} );

disp(metrics_table);

%% ============================ 9) FUZZY MEMBERSHIP FUNCTIONS PLOTS (FOR REPORT) ============================
centers = linspace(-1,1,7);
w_in = centers(2)-centers(1);
x = linspace(-1,1,1201);
labels_in = {'NB','NM','NS','Z','PS','PM','PB'};

% Input MFs (same for e_n and de_n)
mu = zeros(7,numel(x));
for k=1:7
    c = centers(k);
    a = c-w_in; b=c; d=c+w_in;
    for ii=1:numel(x)
        mu(k,ii) = trimf_local(x(ii),a,b,d);
    end
end

figure('Color','w'); hold on; grid on;
for k=1:7
    plot(x,mu(k,:),'LineWidth',1.3);
end
xlabel('x in [-1,1]'); ylabel('mu(x)');
title('Input Membership Functions (for e_n or de_n)');
legend(labels_in,'Location','best');

% Output MFs (gain scalers alpha)
outCenters = [0.10 0.30 0.50 0.70 0.90];
outW = 0.20;
u = linspace(0,1,1201);
labels_out = {'VS','S','M','L','VL'};

muo = zeros(5,numel(u));
for k=1:5
    c = outCenters(k);
    a = max(0,c-outW); b=c; d=min(1,c+outW);
    for ii=1:numel(u)
        muo(k,ii) = trimf_local(u(ii),a,b,d);
    end
end

figure('Color','w'); hold on; grid on;
for k=1:5
    plot(u,muo(k,:),'LineWidth',1.3);
end
xlabel('alpha in [0,1]'); ylabel('mu(alpha)');
title('Output Membership Functions for Gain Scalers (alpha)');
legend(labels_out,'Location','best');