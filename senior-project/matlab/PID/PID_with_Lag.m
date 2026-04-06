%% PID_3R_Zlink_YawPitchPitch_FULL_LAG_FIXANIM.m
% 3R arm: Joint1 yaw about z, Link1 along z (d1), joints 2&3 pitch in vertical plane
% Task-space PID + inverse dynamics + payload cube
% + Actuator Lag (1st-order) + optional torque saturation + simple anti-windup
% White plots + animation with live path drawing
% FIX: animation captured from its own figure handle (no gcf), so no plots appear behind + no getframe errors

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
P.d1 = 0.20;     % [m] first link along z
P.g  = 9.81;

% Dynamics assumptions
P.m1  = 2.5;                 % [kg]
P.m2  = 2.0;                 % [kg]
P.mp  = 0.5;                 % [kg] payload
P.lc1 = P.L1/2;
P.lc2 = P.L2/2;
P.I1  = (1/12)*P.m1*P.L1^2;
P.J1  = 0.15;                % yaw inertia lumped
P.b   = [0.03;0.04;0.02];    % viscous friction

dt = 0.002;
P.dt = dt;

%% ============================ 1.5) ACTUATOR MODEL (LAG + SATURATION) ============================
P.act_enable = true;                 % actuator lag on/off
P.Tact = [0.03; 0.05; 0.05];         % [s] time constants per joint

P.sat_enable = true;                 % saturation on/off
P.tau_max = [25; 30; 20];            % [N.m] max torque magnitude per joint

%% ============================ 2) PICK & PLACE TASK ============================
p_start = [0.55;  0.00; 0.25];
p_pick  = [0.45;  0.18; 0.20];
p_place = [0.40; -0.20; 0.20];
z_lift  = 0.40;

Ts    = [2.0 1.0 2.5 1.0 2.0];
T_end = sum(Ts);
Tmin  = min(Ts);

%% ============================ 3) PID GAINS (2nd-order design) ============================
zeta = 0.9;
wn = 6 / Tmin;

Kp = (wn^2) * eye(3);
Kd = (2*zeta*wn) * eye(3);
Ki = (0.08*wn^3) * eye(3);

ei_max = 0.10;   % integrator clamp

fprintf('\n=== Task-Space PID Gain Design ===\n');
fprintf('Tmin = %.3f s, wn = %.3f rad/s, zeta = %.2f\n', Tmin, wn, zeta);
fprintf('Theoretical Ts(2%%) ? %.3f s\n', 4/(zeta*wn));

%% ============================ 4) INITIAL CONDITIONS ============================
q  = ik_geom_Zlink(p_start,P,'down');
dq = zeros(3,1);
ei = zeros(3,1);

tau_act = zeros(3,1);   % actuator state (actual applied torque)

N = floor(T_end/dt)+1;

% Logs
log.t       = zeros(1,N);
log.p       = zeros(3,N);
log.pd      = zeros(3,N);
log.e       = zeros(3,N);
log.tau     = zeros(3,N);     % store ACTUAL torque (tau_act)
log.tau_cmd = zeros(3,N);
log.tau_act = zeros(3,N);
log.sig     = zeros(1,N);
log.seg     = zeros(1,N);
log.pay     = zeros(1,N);
log.q       = zeros(3,N);
log.qd      = zeros(3,N);

%% ============================ 5) MAIN SIMULATION ============================
for k = 1:N
    t = (k-1)*dt;

    [pd, dpd, ddpd, seg_id] = pickplace_cubic(t,Ts,p_start,p_pick,p_place,z_lift);

    % Payload ON after pick until place (segments 2..4)
    payload_on = (seg_id >= 2) && (seg_id <= 4);

    % Kinematics
    p  = fk_pos_Zlink(q,P);
    J  = jacobian_pos_Zlink(q,P);
    dp = J*dq;

    e  = pd - p;
    de = dpd - dp;

    % Cartesian accel command
    ddp_cmd = ddpd + Kd*de + Kp*e + Ki*ei;

    % Jdot numerical
    eps = 1e-6;
    J2 = jacobian_pos_Zlink(q + dq*eps, P);
    Jdot = (J2 - J)/eps;

    % DLS mapping
    lambda = 1e-4;
    qdd_cmd = (J'*J + lambda*eye(3)) \ (J'*(ddp_cmd - Jdot*dq));

    % Inverse dynamics -> tau_cmd
    [M,C,G] = dyn_MCG_payload(q,dq,P,payload_on);
    tau_cmd = M*qdd_cmd + C*dq + G;

    % Saturation
    if P.sat_enable
        tau_cmd = max(-P.tau_max, min(P.tau_max, tau_cmd));
    end

    % Actuator lag
    if P.act_enable
        tau_act = tau_act + (dt ./ P.Tact) .* (tau_cmd - tau_act);
    else
        tau_act = tau_cmd;
    end

    % Forward dynamics uses ACTUAL torque
    qdd = M \ (tau_act - C*dq - G);
    dq  = dq + qdd*dt;
    q   = q  + dq*dt;

    % Anti-windup: freeze integrator if saturated
    sat_now = false;
    if P.sat_enable
        sat_now = any(abs(tau_cmd) >= (P.tau_max - 1e-9));
    end
    if ~sat_now
        ei = ei + e*dt;
        ei = max(-ei_max, min(ei_max, ei));
    end

    % Desired joint angles for plotting
    qd = ik_geom_Zlink(pd,P,'down');

    % Conditioning
    s = svd(J);
    sigmin = s(end);

    % Log
    log.t(k)        = t;
    log.p(:,k)      = p;
    log.pd(:,k)     = pd;
    log.e(:,k)      = e;
    log.tau_cmd(:,k)= tau_cmd;
    log.tau_act(:,k)= tau_act;
    log.tau(:,k)    = tau_act;
    log.sig(k)      = sigmin;
    log.seg(k)      = seg_id;
    log.pay(k)      = payload_on;
    log.q(:,k)      = q;
    log.qd(:,k)     = qd;
end

%% ============================ 6) PLOTS ============================
% Path
figure('Color','w'); hold on; grid on; axis equal;
plot3(log.pd(1,:),log.pd(2,:),log.pd(3,:),'k--','LineWidth',1.6);
plot3(log.p(1,:), log.p(2,:), log.p(3,:), 'b','LineWidth',1.6);
plot3(p_start(1),p_start(2),p_start(3),'bo','MarkerSize',8,'MarkerFaceColor','b');
plot3(p_pick(1), p_pick(2), p_pick(3), 'go','MarkerSize',8,'MarkerFaceColor','g');
plot3(p_place(1),p_place(2),p_place(3),'mo','MarkerSize',8,'MarkerFaceColor','m');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('Desired Path','Actual Path','Start','Pick','Place','Location','best');
title('Pick-and-Place Cartesian Path');

% Errors
figure('Color','w');
subplot(3,1,1); plot(log.t,log.e(1,:),'LineWidth',1.2); grid on; ylabel('e_x (m)');
subplot(3,1,2); plot(log.t,log.e(2,:),'LineWidth',1.2); grid on; ylabel('e_y (m)');
subplot(3,1,3); plot(log.t,log.e(3,:),'LineWidth',1.2); grid on; ylabel('e_z (m)'); xlabel('Time (s)');
my_sgtitle('Task-Space Tracking Errors');

% Torques (cmd vs act)
figure('Color','w');
subplot(3,1,1);
plot(log.t,log.tau_cmd(1,:),'--','LineWidth',1.0); hold on;
plot(log.t,log.tau_act(1,:),'LineWidth',1.1);
grid on; ylabel('\tau_1 (N.m)'); legend('cmd','act','Location','best');

subplot(3,1,2);
plot(log.t,log.tau_cmd(2,:),'--','LineWidth',1.0); hold on;
plot(log.t,log.tau_act(2,:),'LineWidth',1.1);
grid on; ylabel('\tau_2 (N.m)'); legend('cmd','act','Location','best');

subplot(3,1,3);
plot(log.t,log.tau_cmd(3,:),'--','LineWidth',1.0); hold on;
plot(log.t,log.tau_act(3,:),'LineWidth',1.1);
grid on; ylabel('\tau_3 (N.m)'); xlabel('Time (s)'); legend('cmd','act','Location','best');
my_sgtitle('Joint Torques (Commanded vs Actual with Actuator Lag)');

% Jacobian conditioning
figure('Color','w');
plot(log.t,log.sig,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('\sigma_{min}(J)'); title('Jacobian Conditioning Indicator');

% Payload indicator
figure('Color','w');
stairs(log.t,log.pay,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Payload ON (0/1)'); title('Payload Activation During Pick-and-Place');

% Joint angles actual vs desired
figure('Color','w');
subplot(3,1,1); plot(log.t,log.q(1,:),'LineWidth',1.2); hold on; plot(log.t,log.qd(1,:),'--','LineWidth',1.2);
grid on; ylabel('q_1 (rad)'); legend('Actual','Desired','Location','best');
subplot(3,1,2); plot(log.t,log.q(2,:),'LineWidth',1.2); hold on; plot(log.t,log.qd(2,:),'--','LineWidth',1.2);
grid on; ylabel('q_2 (rad)'); legend('Actual','Desired','Location','best');
subplot(3,1,3); plot(log.t,log.q(3,:),'LineWidth',1.2); hold on; plot(log.t,log.qd(3,:),'--','LineWidth',1.2);
grid on; ylabel('q_3 (rad)'); xlabel('Time (s)'); legend('Actual','Desired','Location','best');
my_sgtitle('Joint Angles Tracking (Actual vs Desired)');

%% ============================ 7) ANIMATION + PATH DRAWING (FIXED: CAPTURE ONLY ANIM FIG) ============================
figAnim = figure('Color','w'); clf(figAnim);
axAnim  = axes('Parent',figAnim); hold(axAnim,'on'); grid(axAnim,'on');
axis(axAnim,'equal'); axis(axAnim,'manual');
xlabel(axAnim,'X (m)'); ylabel(axAnim,'Y (m)'); zlabel(axAnim,'Z (m)');
view(axAnim,45,25);
title(axAnim,'3R Arm Animation (Link1 along Z) + Live Path');
xlim(axAnim,[-0.8 0.8]); ylim(axAnim,[-0.8 0.8]); zlim(axAnim,[0 0.7]);

% base
plot3(axAnim,0,0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');

% handles
h_l0 = plot3(axAnim,[0 0],[0 0],[0 0],'b','LineWidth',3);
h_l1 = plot3(axAnim,[0 0],[0 0],[0 0],'r','LineWidth',3);
h_l2 = plot3(axAnim,[0 0],[0 0],[0 0],'g','LineWidth',3);
h_ee = plot3(axAnim,0,0,0,'ro','MarkerSize',6,'MarkerFaceColor','r');

h_path_act = plot3(axAnim,NaN,NaN,NaN,'b','LineWidth',1.6);
h_path_des = plot3(axAnim,NaN,NaN,NaN,'k--','LineWidth',1.2);

cube_size = 0.04;
h_cube = patch('Parent',axAnim, ...
    'Faces',[1 2 3 4 5 6 7 8], ...
    'Vertices',cube_vertices(p_pick,cube_size), ...
    'FaceColor',[0.2 0.8 0.3],'EdgeColor','k');

Xa=[]; Ya=[]; Za=[];
Xd=[]; Yd=[]; Zd=[];

stepAnim = 10;
gifFile = 'pick_and_place_animation.gif';
gifDelay = 0.03;
isFirstFrame = true;

for kk = 1:stepAnim:length(log.t)

    qk = log.q(:,kk);
    q1 = qk(1); q2 = qk(2); q3 = qk(3);

    p0 = [0;0;0];
    p1 = [0;0;P.d1];

    r2 = P.L1*cos(q2);
    z2 = P.d1 + P.L1*sin(q2);
    p2 = [cos(q1)*r2; sin(q1)*r2; z2];

    r3 = P.L1*cos(q2) + P.L2*cos(q2+q3);
    z3 = P.d1 + P.L1*sin(q2) + P.L2*sin(q2+q3);
    p3 = [cos(q1)*r3; sin(q1)*r3; z3];

    set(h_l0,'XData',[p0(1) p1(1)],'YData',[p0(2) p1(2)],'ZData',[p0(3) p1(3)]);
    set(h_l1,'XData',[p1(1) p2(1)],'YData',[p1(2) p2(2)],'ZData',[p1(3) p2(3)]);
    set(h_l2,'XData',[p2(1) p3(1)],'YData',[p2(2) p3(2)],'ZData',[p2(3) p3(3)]);
    set(h_ee,'XData',p3(1),'YData',p3(2),'ZData',p3(3));

    if log.pay(kk)==1
        set(h_cube,'Vertices',cube_vertices(p3,cube_size));
    else
        if log.seg(kk) < 2
            set(h_cube,'Vertices',cube_vertices(p_pick,cube_size));
        else
            set(h_cube,'Vertices',cube_vertices(p_place,cube_size));
        end
    end

    Xa(end+1)=p3(1); Ya(end+1)=p3(2); Za(end+1)=p3(3);
    Xd(end+1)=log.pd(1,kk); Yd(end+1)=log.pd(2,kk); Zd(end+1)=log.pd(3,kk);
    set(h_path_act,'XData',Xa,'YData',Ya,'ZData',Za);
    set(h_path_des,'XData',Xd,'YData',Yd,'ZData',Zd);

    drawnow limitrate;
    pause(0.01);

    % IMPORTANT: capture ONLY the animation axes (most robust)
    frame = getframe(axAnim);
    img = frame2im(frame);
    [imind, cm] = rgb2ind(img, 256);

    if isFirstFrame
        imwrite(imind, cm, gifFile, 'gif', 'Loopcount', inf, 'DelayTime', gifDelay);
        isFirstFrame = false;
    else
        imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
    end
end

%% ============================ 5.5) METRICS EXTRACTION ============================
t = log.t(:);
e = log.e.';        % Nx3
tau = log.tau.';    % Nx3 (ACTUAL torque)
pay = log.pay(:) > 0;
sig = log.sig(:);

Ttotal = t(end) - t(1);
ss_idx = t >= (t(1) + 0.90*Ttotal);

pay_idx = pay;
nopay_idx = ~pay;

e_abs = abs(e);
e_rms = sqrt(mean(e.^2, 1));
e_max = max(e_abs, [], 1);
e_ss_abs_mean = mean(abs(e(ss_idx, :)), 1);

if any(pay_idx)
    e_rms_pay = sqrt(mean(e(pay_idx,:).^2, 1));
    e_max_pay = max(abs(e(pay_idx,:)), [], 1);
else
    e_rms_pay = [NaN NaN NaN];
    e_max_pay = [NaN NaN NaN];
end

e_norm = vecnorm(e,2,2);
e_norm_rms = sqrt(mean(e_norm.^2));
e_norm_max = max(e_norm);

if any(pay_idx)
    e_norm_rms_pay = sqrt(mean(e_norm(pay_idx).^2));
    e_norm_max_pay = max(e_norm(pay_idx));
else
    e_norm_rms_pay = NaN;
    e_norm_max_pay = NaN;
end

tau_abs = abs(tau);
tau_rms = sqrt(mean(tau.^2, 1));
tau_max = max(tau_abs, [], 1);

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

fprintf('\n==================== METRICS SUMMARY ====================\n');
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

fprintf('\nJoint Torque RMS [t1 t2 t3] (N.m) (ACT): %.3e  %.3e  %.3e\n', tau_rms(1), tau_rms(2), tau_rms(3));
fprintf('Joint Torque MAX [t1 t2 t3] (N.m) (ACT): %.3e  %.3e  %.3e\n', tau_max(1), tau_max(2), tau_max(3));

if any(pay_idx) && any(nopay_idx)
    fprintf('\nTorque increase due to payload (RMS) %%:  %.2f%%  %.2f%%  %.2f%%\n', tau_rms_inc_pct(1), tau_rms_inc_pct(2), tau_rms_inc_pct(3));
    fprintf('Torque increase due to payload (MAX) %%:  %.2f%%  %.2f%%  %.2f%%\n', tau_max_inc_pct(1), tau_max_inc_pct(2), tau_max_inc_pct(3));
end

fprintf('\nJacobian sigma_min(J):                   %.3e\n', sig_min);
fprintf('Jacobian mean sigma_min(J):              %.3e\n', sig_mean);
fprintf('=========================================================\n');

metrics_table = table( ...
    e_rms(:), e_max(:), e_ss_abs_mean(:), e_rms_pay(:), e_max_pay(:), ...
    tau_rms(:), tau_max(:), tau_rms_pay(:), tau_max_pay(:), ...
    'VariableNames', {'e_RMS','e_MAX','e_SS_abs_mean','e_RMS_payload','e_MAX_payload', ...
                      'tau_RMS_ACT','tau_MAX_ACT','tau_RMS_ACT_payload','tau_MAX_ACT_payload'}, ...
    'RowNames', {'X','Y','Z'} );
disp(metrics_table);


