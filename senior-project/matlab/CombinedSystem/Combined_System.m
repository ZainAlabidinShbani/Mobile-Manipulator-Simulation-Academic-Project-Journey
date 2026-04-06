%% omni_YPP_pick_place_PID_APF_robust_ONEFILE_WITH_LIDAR_MAP_UI__ARM_JOINT_PID.m
% Same as your script, ONLY change: Arm control = Joint PID (with anti-windup) instead of nonlinear tanh PD.
% Everything else (APF, mapping, UI, animation) stays the same.
%
% ====== NOTE (Modified as requested): ======
% Removed obstacle UI + pick/place UI. Using defaults only.

clear; clc; close all;
rng(0); % reproducible

%% ===================== Simulation =====================
dt   = 0.01;
Tend = 32;
t    = 0:dt:Tend;
N    = numel(t);

%% ===================== World =====================
world.xlim = [-0.2 3.4];
world.ylim = [-0.2 2.6];
world.zlim = [ 0.0 1.0];

%% ===================== Obstacles (DEFAULT ONLY) =====================
obstacles_default = [
    1.30  0.90  0.25
    2.20  1.60  0.30
    2.00  0.40  0.22
    0.90  1.70  0.25
];

% UI REMOVED -> always use defaults
obstacles = obstacles_default;

%% ===================== Pick & Place Setup (DEFAULT ONLY) =====================
obj.pick_xy  = [0.85; 0.55];
obj.place_xy = [2.85; 2.05];
obj.z_table  = 0.08;
obj.size     = 0.06;

% UI REMOVED -> keep defaults above (no mouse selection)

base.pick_goal  = obj.pick_xy  + [-0.28; -0.18];
base.place_goal = obj.place_xy + [-0.28; -0.18];

ee_home = [0.55; 0.08; 0.30];

z_approach = obj.z_table + 0.22;
z_pick     = obj.z_table + obj.size*0.55;
z_place    = obj.z_table + obj.size*0.55;

%% ===================== Base Control =====================
vmax = 0.85;
wmax = 1.70;

pid_x  = struct('Kp',2.8,'Ki',0.08,'Kd',0.35,'I',0,'prevE',0);
pid_y  = struct('Kp',2.8,'Ki',0.08,'Kd',0.35,'I',0,'prevE',0);
pid_th = struct('Kp',3.8,'Ki',0.05,'Kd',0.28,'I',0,'prevE',0);

apf.k_att       = 2.2;
apf.k_rep       = 1.4;
apf.d0          = 0.85;
apf.inflation   = 0.10;
apf.safe_margin = 0.06;
apf.swirl_k     = 0.55;
apf.v_apf_max   = 0.85;
apf.hard_stop_d = 0.05;
apf.slow_d      = 0.35;

base.x  = 0.25; base.y = 0.25; base.th = 0.0;
base_reach_xy = 0.10;
base_reach_th = 0.18;

%% ===================== Arm YPP (3D) =====================
arm.L  = [0.45; 0.35; 0.25];
arm.q  = deg2rad([20; -20; 20]);
arm.qd = zeros(3,1);

% simplified dynamics params (same)
arm.J = [0.8; 0.6; 0.45];
arm.b = [0.13; 0.11; 0.09];

% IK
ik.lambda = 0.12;
ik.iters  = 15;
ik.dq_max = deg2rad(12);

% ====== ARM JOINT PID (NEW) ======
% (These are reasonable starting gains; stable + decent tracking)
pid_q(1) = struct('Kp',14.0,'Ki',1.2,'Kd',1.6,'I',0,'prevE',0,'Imax',0.8);
pid_q(2) = struct('Kp',16.0,'Ki',1.4,'Kd',1.8,'I',0,'prevE',0,'Imax',0.8);
pid_q(3) = struct('Kp',12.0,'Ki',1.0,'Kd',1.4,'I',0,'prevE',0,'Imax',0.8);

tau_max  = [10; 8; 7];

% optional extra stabilizing damping (prevents oscillations)
virt_damp = [0.35; 0.30; 0.25];  % set to zeros(3,1) if you want

% optional: smooth EE target to remove z-step shocks (recommended)
useRefFilter = true;
ee_rel_ref = ee_home;
tau_ref = 0.25; % seconds (bigger = smoother)

%% ===================== State Machine =====================
PH.MOVE_TO_PICK      = 1;
PH.ARM_APPROACH_PICK = 2;
PH.ARM_DESCEND_PICK  = 3;
PH.GRASP_LIFT        = 4;
PH.MOVE_TO_PLACE     = 5;
PH.ARM_DESCEND_PLACE = 6;
PH.RELEASE_RETRACT   = 7;
PH.DONE              = 8;

phase = PH.MOVE_TO_PICK;
gripped = false;
phase_timer = 0;

obj_w = [obj.pick_xy; obj.z_table + obj.size*0.5];

%% ===================== LiDAR + Mapping =====================
lidar.fov      = 2*pi;
lidar.nRays    = 200;
lidar.rMax     = 3.5;
lidar.sigmaR   = 0.01;
lidar.hitProb  = 0.72;
lidar.freeProb = 0.45;
lidar.zHit     = 0.06;

map.res  = 0.04;
map.xlim = world.xlim;
map.ylim = world.ylim;
map.nx = ceil((map.xlim(2)-map.xlim(1))/map.res);
map.ny = ceil((map.ylim(2)-map.ylim(1))/map.res);

map.L     = zeros(map.ny, map.nx);
map.L_occ = log(lidar.hitProb/(1-lidar.hitProb));
map.L_free= log(lidar.freeProb/(1-lidar.freeProb));
map.Lmin  = -6;
map.Lmax  =  6;

%% ===================== Logging =====================
log.base   = zeros(N,3);
log.base_d = zeros(N,3);
log.vcmd   = zeros(N,3);
log.err    = zeros(N,3);

log.q      = zeros(N,3);
log.ee     = zeros(N,3);
log.ee_d   = zeros(N,3);

log.phase  = zeros(N,1);
log.obj    = zeros(N,3);

log.lidar  = cell(N,1);
log.mapP   = cell(N,1);

%% ===================== Main Loop =====================
for k = 1:N
    % ========= Decide goals (base + arm) =========
    switch phase
        case {PH.MOVE_TO_PICK, PH.ARM_APPROACH_PICK, PH.ARM_DESCEND_PICK, PH.GRASP_LIFT}
            base_goal_xy = base.pick_goal;
            look_xy = obj.pick_xy;
        case {PH.MOVE_TO_PLACE, PH.ARM_DESCEND_PLACE, PH.RELEASE_RETRACT}
            base_goal_xy = base.place_goal;
            look_xy = obj.place_xy;
        otherwise
            base_goal_xy = [base.x; base.y];
            look_xy = [base.x; base.y];
    end

    thd = wrapToPi(atan2(look_xy(2)-base.y, look_xy(1)-base.x));
    xd = base_goal_xy(1); yd = base_goal_xy(2);

    % ========= Base Robust APF velocity =========
    [v_apf, dmin] = apf_velocity_robust([base.x; base.y], [xd; yd], obstacles, apf);

    % PID tracking
    [vx_pid, pid_x, ex]   = pid_step(xd, base.x, pid_x, dt, false);
    [vy_pid, pid_y, ey]   = pid_step(yd, base.y, pid_y, dt, false);
    [w_pid,  pid_th, eth] = pid_step(thd, base.th, pid_th, dt, true);

    hold_base = ismember(phase, [PH.ARM_DESCEND_PICK, PH.ARM_DESCEND_PLACE]);
    hold_gain = 0.15;

    if hold_base
        vx = hold_gain*(vx_pid + v_apf(1));
        vy = hold_gain*(vy_pid + v_apf(2));
        w  = hold_gain*(w_pid);
    else
        vx = vx_pid + v_apf(1);
        vy = vy_pid + v_apf(2);
        w  = w_pid;
    end

    if dmin < apf.slow_d
        s = max(0.15, dmin / apf.slow_d);
        vx = vx*s; vy = vy*s; w = w*s;
    end

    vxy = hypot(vx,vy);
    if vxy > vmax
        sc = vmax / max(vxy,1e-9);
        vx = vx*sc; vy = vy*sc;
    end
    w = max(min(w, wmax), -wmax);

    [base, vx, vy] = integrate_base_with_shield(base, vx, vy, w, dt, obstacles, apf);

    % ========= LiDAR scan + mapping =========
    pose = [base.x; base.y; base.th];
    [ranges, ptsW] = lidar_scan_2d(pose, obstacles, apf, lidar);
    map = occgrid_update(map, pose, ranges, lidar);
    log.lidar{k} = ptsW;

    % ========= Arm desired EE world target =========
    switch phase
        case PH.ARM_APPROACH_PICK
            ee_world_target = [obj.pick_xy; z_approach];
        case PH.ARM_DESCEND_PICK
            ee_world_target = [obj.pick_xy; z_pick];
        case PH.GRASP_LIFT
            ee_world_target = [obj.pick_xy; z_approach];
        case PH.ARM_DESCEND_PLACE
            ee_world_target = [obj.place_xy; z_place];
        case PH.RELEASE_RETRACT
            ee_world_target = [obj.place_xy; z_approach];
        otherwise
            ee_world_target = [base.x; base.y; 0] + rotz(base.th)*ee_home;
    end

    % Convert EE desired to base frame
    Rz = rotz(base.th);
    ee_rel_d = Rz' * (ee_world_target - [base.x; base.y; 0]);

    % Move-phase blending
    if phase == PH.MOVE_TO_PICK || phase == PH.MOVE_TO_PLACE
        alpha = 0.22;
        ee_rel_d = (1-alpha)*ee_home + alpha*ee_rel_d;
    end

    % Clamp reachable-ish region
    ee_rel_d(1) = max(min(ee_rel_d(1), sum(arm.L)-0.06), 0.18);
    ee_rel_d(3) = max(min(ee_rel_d(3), 0.70), 0.06);

    % OPTIONAL reference smoothing (recommended)
    if useRefFilter
        a = dt/(tau_ref+dt);
        ee_rel_ref = (1-a)*ee_rel_ref + a*ee_rel_d;
        ee_rel_d = ee_rel_ref;
    end

    % ========= IK (DLS) -> q_des (with adaptive damping near singularity) =========
    Jtmp = jacobian_num(arm.q, arm.L);
    ssv  = svd(Jtmp);
    lambda_eff = ik.lambda;
    if min(ssv) < 0.05
        lambda_eff = 0.25;
    elseif min(ssv) < 0.10
        lambda_eff = 0.18;
    end

    q_des = ik_3r_ypp_dls(arm.q, ee_rel_d, arm.L, lambda_eff, ik.iters, ik.dq_max);

    % ========= Arm Joint PID (NEW) =========
    tau = zeros(3,1);
    for i=1:3
        [tau(i), pid_q(i)] = pid_step_aw(q_des(i), arm.q(i), pid_q(i), dt, true, tau_max(i));
    end

    % ========= integrate dynamics (same model) =========
    qdd = (tau - (arm.b + virt_damp) .* arm.qd) ./ arm.J;

    arm.qd = arm.qd + qdd*dt;
    arm.q  = wrapToPi(arm.q + arm.qd*dt);

    % ========= FK EE =========
    ee_rel   = fk_3r_ypp(arm.q, arm.L);
    ee_world = [base.x; base.y; 0] + Rz*ee_rel;

    % ========= Object attach =========
    if gripped
        obj_w = ee_world + [0;0; -obj.size*0.35];
    end

    % ========= Phase transitions =========
    phase_timer = phase_timer + dt;

    base_ok = (hypot(base.x-xd, base.y-yd) < base_reach_xy) && ...
              (abs(wrapToPi(thd-base.th)) < base_reach_th);

    if ismember(phase,[PH.ARM_APPROACH_PICK, PH.GRASP_LIFT, PH.RELEASE_RETRACT])
        ee_tol = 0.055;
    else
        ee_tol = 0.040;
    end
    ee_ok = norm(ee_world - ee_world_target) < ee_tol;

    switch phase
        case PH.MOVE_TO_PICK
            if base_ok || phase_timer > 10.5
                phase = PH.ARM_APPROACH_PICK; phase_timer = 0;
            end
        case PH.ARM_APPROACH_PICK
            if ee_ok || phase_timer > 2.2
                phase = PH.ARM_DESCEND_PICK; phase_timer = 0;
            end
        case PH.ARM_DESCEND_PICK
            if ee_ok || phase_timer > 2.4
                gripped = true;
                phase = PH.GRASP_LIFT; phase_timer = 0;
            end
        case PH.GRASP_LIFT
            if ee_ok || phase_timer > 2.2
                phase = PH.MOVE_TO_PLACE; phase_timer = 0;
            end
        case PH.MOVE_TO_PLACE
            if base_ok || phase_timer > 12.5
                phase = PH.ARM_DESCEND_PLACE; phase_timer = 0;
            end
        case PH.ARM_DESCEND_PLACE
            if ee_ok || phase_timer > 2.4
                gripped = false;
                obj_w = [obj.place_xy; obj.z_table + obj.size*0.5];
                phase = PH.RELEASE_RETRACT; phase_timer = 0;
            end
        case PH.RELEASE_RETRACT
            if ee_ok || phase_timer > 2.0
                phase = PH.DONE; phase_timer = 0;
            end
    end

    % ========= Log =========
    log.base(k,:)   = [base.x base.y base.th];
    log.base_d(k,:) = [xd yd thd];
    log.vcmd(k,:)   = [vx vy w];
    log.err(k,:)    = [ex ey eth];

    log.q(k,:)      = arm.q';
    log.ee(k,:)     = ee_world';
    log.ee_d(k,:)   = ee_world_target';

    log.phase(k)    = phase;
    log.obj(k,:)    = obj_w';

    if mod(k, round(0.30/dt))==0
        Pm = 1 - 1./(1+exp(map.L));
        log.mapP{k} = Pm;
    end
end

%% ===================== Plots =====================
plot_results(t, log, obstacles, world);

%% ===================== Final Map Plot =====================
figure('Name','Final Occupancy Map');
P = 1 - 1./(1+exp(map.L));
imagesc(linspace(map.xlim(1),map.xlim(2),map.nx), linspace(map.ylim(1),map.ylim(2),map.ny), P);
set(gca,'YDir','normal'); axis equal tight; colormap(gray);
hold on;
plot(log.base(:,1), log.base(:,2), 'LineWidth',1.2);
title('Occupancy probability (0 free .. 1 occupied)');
xlabel('x'); ylabel('y');

%% ===================== Animation (Wide + Map + Short obstacles) =====================
animate_scene_3d_wide_with_map(t, log, obstacles, world, arm.L, obj.size, map, lidar);
%% ===================== Animation + Save Video =====================
saveVideo = true;                         % Õÿ false ≈–« »œﬂ ⁄—÷ ›ﬁÿ »œÊ‰ Õ›Ÿ
videoName = 'robot_pick_place.mp4';       % «”„ «·›ÌœÌÊ
fps = 30;                                 % ”—⁄… «·›ÌœÌÊ

fig = figure('Name','Animation Recording');

if saveVideo
    v = VideoWriter(videoName,'MPEG-4');
    v.FrameRate = fps;
    open(v);
end

for k = 1:length(t)

    clf(fig);

    % ‰—”„ «·„‘Âœ ··ÕŸ… «·Õ«·Ì…
    animate_scene_3d_wide_with_map(t(1:k), log, obstacles, world, arm.L, obj.size, map, lidar);

    drawnow;

    if saveVideo
        frame = getframe(fig);
        writeVideo(v, frame);
    end
end

if saveVideo
    close(v);
    disp(['Video saved as: ', videoName]);
end