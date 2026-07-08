classdef MobileManipulatorEnv < rl.env.MATLABEnvironment
% MOBILEMANIPULATORENV  Custom RL environment for a 3R robotic arm
% mounted on a 4-wheel mobile base performing pick-and-place tasks.
%
% Compatible with: MATLAB R2023b + Reinforcement Learning Toolbox
%
% Observation (14-D, normalised):
%   [q1 q2 q3 | dq1 dq2 dq3 | px py pz | x_base theta_base | dx dz | grip]
%
% Action (5-D continuous):
%   [dq1_dot dq2_dot dq3_dot | vx omega]
%
% Key fixes vs. report code:
%   - clamp()  replaced with  min(max())  (no built-in clamp for arrays)
%   - Learning-rate typos fixed: 3e4->3e-4, 1e3->1e-3
%   - reset() returns [obs, loggedSignals] per R2022a+ API
%   - All positions unified in world frame; FK accounts for base rotation
%   - Pick/Goal zones placed within arm reach (< 0.30 m from shoulder)

    % ----------------------------------------------------------------
    properties   % public: readable from training / evaluation scripts
        % ---- Physical parameters ----
        L  = [0.12; 0.10; 0.08];        % Link lengths  L1 L2 L3 [m]
        m  = [5.0;  0.8;  0.5;  0.3];  % Masses: base, link1, link2, link3 [kg]
        ShoulderHeight = 0.20;           % Shoulder above base centre [m]
        HalfTrackWidth = 0.12;           % Half track width  d  [m]
        HalfBaseLen    = 0.14;           % Half base length      [m]

        % Joint limits  [min max]  rad, one row per joint
        JointLimits = [-pi/2,  pi/2  ; ...
                        0,     3*pi/4 ; ...
                       -pi/2,  pi/2  ];

        MaxJointVel = [1.5; 1.5; 2.0];  % [rad/s]
        MaxBaseVel  = [0.3; 0.5];        % [vx m/s ; omega rad/s]
        Ts          = 0.05;              % Control period [s] (20 Hz)

        % ---- Task parameters (world frame [m]) ----
        % Both zones are within arm reach once the base has navigated close.
        % Pick: robot must drive ~0.30 m forward; arm then extends 0.22 m.
        % Goal: offset laterally so base rotation + arm are both needed.
        GoalZone = [0.20; 0.30; 0.10];  % Fixed place position (world)
        GraspTol = 0.03;                 % Grasp success threshold [m]
        PlaceTol = 0.05;                 % Place success threshold [m]
        MaxSteps = 200;                  % Episode length (10 s at 20 Hz)

        % ---- Mutable episode state (handle class -> in-place updates) ----
        q             = zeros(3,1);   % Joint angles     [rad]
        qdot          = zeros(3,1);   % Joint velocities  [rad/s]
        BasePose      = [0;0;0];      % [x; y; theta]    [m m rad]
        GripperClosed = 0;
        ObjectGrasped = 0;
        ObjectPos     = [0.50; 0.00; 0.10]; % World-frame object position
        StepCount     = 0;

        % Reward-component log (public so evaluation script can read them)
        Rc_reach  = 0;  Rc_orient = 0;  Rc_grasp = 0;
        Rc_carry  = 0;  Rc_place  = 0;  Rc_jlim  = 0;
        Rc_tip    = 0;  Rc_smooth = 0;
    end

    % ----------------------------------------------------------------
    methods

        function env = MobileManipulatorEnv()
            % Observation spec: 14 x 1, normalised -> keep limits at inf
            obsInfo = rlNumericSpec([14 1], ...
                'LowerLimit', -inf(14,1), 'UpperLimit', inf(14,1));
            obsInfo.Name = 'MobileManipulatorObs';

            % Action spec: 5 x 1, physical bounds enforced here
            actInfo = rlNumericSpec([5 1], ...
                'LowerLimit', [-1.5;-1.5;-2.0;-0.3;-0.5], ...
                'UpperLimit', [ 1.5; 1.5; 2.0; 0.3; 0.5]);
            actInfo.Name = 'MobileManipulatorAct';

            env = env@rl.env.MATLABEnvironment(obsInfo, actInfo);
            env.ObjectPos = [0.50; 0.00; 0.10];   % default; reset randomises
        end

        % ---- STEP -------------------------------------------------------
        function [nextObs, reward, isDone, loggedSignals] = step(env, action)
            loggedSignals = [];

            % Hard-clip action to physical limits
            aLo = [-1.5;-1.5;-2.0;-0.3;-0.5];
            aHi = [ 1.5; 1.5; 2.0; 0.3; 0.5];
            action = min(max(action(:), aLo), aHi);

            % 1. Integrate joint velocities (Euler), clip to limits
            env.q    = min(max(env.q + action(1:3)*env.Ts, ...
                               env.JointLimits(:,1)), ...
                               env.JointLimits(:,2));
            env.qdot = action(1:3);

            % 2. Unicycle base kinematics
            vx  = action(4);  om = action(5);
            th  = env.BasePose(3);
            env.BasePose(1) = env.BasePose(1) + vx*cos(th)*env.Ts;
            env.BasePose(2) = env.BasePose(2) + vx*sin(th)*env.Ts;
            env.BasePose(3) = env.BasePose(3) + om*env.Ts;

            % 3. Forward kinematics -> EE in both frames
            p_s = env.fwdKin();           % shoulder frame
            p_w = env.eeWorld(p_s);       % world frame

            % 4. Grasp detection: object snaps to EE if close enough
            d_obj = norm(env.ObjectPos - p_w);
            justGrasped = false;
            if ~env.ObjectGrasped && d_obj < env.GraspTol
                env.GripperClosed = 1;
                env.ObjectGrasped = 1;
                justGrasped       = true;
            end
            if env.ObjectGrasped
                env.ObjectPos = p_w;    % object follows EE in world frame
            end

            % 5. Multi-component reward
            reward = env.calcReward(p_w, action, d_obj, justGrasped);

            % 6. Termination
            env.StepCount = env.StepCount + 1;
            placed  = env.ObjectGrasped && ...
                      norm(env.ObjectPos - env.GoalZone) < env.PlaceTol;
            isDone  = logical(placed || ...
                              env.StepCount >= env.MaxSteps || ...
                              env.jLimViolated() || ...
                              env.tipoverRisk());

            % 7. Build observation
            nextObs = env.buildObs(p_s, p_w);
        end

        % ---- RESET ------------------------------------------------------
        % Returns [obs, loggedSignals] per R2022a+ rl.env.MATLABEnvironment API
        function [obs, loggedSignals] = reset(env)
            loggedSignals = [];
            env.q             = zeros(3,1);
            env.qdot          = zeros(3,1);
            env.BasePose      = [0; 0; 0];
            env.GripperClosed = 0;
            env.ObjectGrasped = 0;
            env.StepCount     = 0;

            % Randomise object position +-5 cm for generalisation
            env.ObjectPos = [0.50 + 0.05*(2*rand-1); ...
                             0.00 + 0.05*(2*rand-1); ...
                             0.10];

            p_s = env.fwdKin();
            p_w = env.eeWorld(p_s);
            obs = env.buildObs(p_s, p_w);
        end

        % ---- Public helpers (callable from evaluate_and_plot.m) ---------
        function p_s = getEEShoulderFrame(env)
            p_s = env.fwdKin();
        end

        function p_w = getEEWorldFrame(env)
            p_w = env.eeWorld(env.fwdKin());
        end

        function com = getCoM(env)
            com = env.computeCoM();
        end

    end   % public methods

    % ----------------------------------------------------------------
    methods (Access = private)

        function p = fwdKin(env)
            % 3R planar FK in arm's XZ plane.
            % Origin = shoulder joint; +X = arm forward; +Z = arm up.
            % At q=[0,0,0] the arm points straight up (+Z).
            x = 0;  z = 0;
            cumAng = 0;
            for k = 1:3
                cumAng = cumAng + env.q(k);
                x = x + env.L(k) * sin(cumAng);
                z = z + env.L(k) * cos(cumAng);
            end
            p = [x; 0; z];   % py = 0 (planar arm)
        end

        function pw = eeWorld(env, p_shoulder)
            % Convert EE from shoulder frame to world frame,
            % accounting for base heading (theta).
            th = env.BasePose(3);
            R  = [cos(th), -sin(th), 0; ...
                  sin(th),  cos(th), 0; ...
                  0,        0,       1];
            shoulder_w = [env.BasePose(1); env.BasePose(2); env.ShoulderHeight];
            pw = shoulder_w + R * p_shoulder;
        end

        function obs = buildObs(env, p_s, p_w)
            % Build + normalise 14-D observation.
            % Division scales each component to approximately [-1, 1].
            dT  = env.ObjectPos - p_w;   % world-frame reach error

            q_n    = env.q    ./ [pi/2; 3*pi/4; pi/2];  % normalise by limits
            qd_n   = env.qdot ./ env.MaxJointVel;
            ps_n   = p_s      ./ 0.35;                  % max arm reach ~0.35 m
            bx_n   = env.BasePose(1) / 2.0;
            bth_n  = env.BasePose(3) / pi;
            dt_n   = [dT(1); dT(3)] / 1.0;             % +-1 m typical range
            g_n    = double(env.GripperClosed);

            obs = [q_n; qd_n; ps_n; bx_n; bth_n; dt_n; g_n];
            % 3+3+3+1+1+2+1 = 14
        end

        function r = calcReward(env, p_w, action, d_obj, justGrasped)
            % === Dense: minimise EE-to-object distance ===
            env.Rc_reach  = -d_obj;

            % === Dense: gripper orientation (wrist angle proxy) ===
            wrist_angle   = sum(env.q);
            target_angle  = pi/6;   % slight forward tilt for grasping
            env.Rc_orient = -0.10 * abs(wrist_angle - target_angle);

            % === Sparse: one-time grasp bonus ===
            env.Rc_grasp  = 0;
            if justGrasped
                env.Rc_grasp = 100.0;
            end

            % === Dense: carry toward goal (active after grasp) ===
            env.Rc_carry  = 0;
            env.Rc_place  = 0;
            if env.ObjectGrasped
                d_goal = norm(env.ObjectPos - env.GoalZone);
                env.Rc_carry = -0.5 * d_goal;
                % Sparse: placement bonus
                if d_goal < env.PlaceTol
                    env.Rc_place = 200.0;
                end
            end

            % === Safety penalties ===
            env.Rc_jlim = 0;
            if env.jLimViolated(),  env.Rc_jlim = -20.0; end

            env.Rc_tip  = 0;
            if env.tipoverRisk(),   env.Rc_tip  = -50.0; end

            % === Dense: action smoothness regularisation ===
            env.Rc_smooth = -0.005 * (action' * action);

            r = env.Rc_reach  + env.Rc_orient + env.Rc_grasp + ...
                env.Rc_carry  + env.Rc_place  + env.Rc_jlim  + ...
                env.Rc_tip    + env.Rc_smooth;
        end

        function hit = jLimViolated(env)
            % True if any joint is beyond its limit by > 5 deg (soft check)
            buf = 5 * pi/180;
            hit = any(env.q < env.JointLimits(:,1) - buf) || ...
                  any(env.q > env.JointLimits(:,2) + buf);
        end

        function risk = tipoverRisk(env)
            % ZMP check: forward CoM offset vs support polygon
            com  = env.computeCoM();   % in shoulder frame
            risk = abs(com(1)) > env.HalfBaseLen * 1.8;
        end

        function com = computeCoM(env)
            % CoM of full system in shoulder frame.
            % Base CoM is ShoulderHeight below the shoulder (directly below).
            totalM = sum(env.m);
            com    = env.m(1) * [0; 0; -env.ShoulderHeight]; % base
            p      = zeros(3,1);
            cumAng = 0;
            for k = 1:3
                cumAng = cumAng + env.q(k);
                dir  = [sin(cumAng); 0; cos(cumAng)];
                mid  = p + 0.5 * env.L(k) * dir;
                com  = com + env.m(k+1) * mid;
                p    = p   + env.L(k)   * dir;
            end
            com = com / totalM;
        end

    end   % private methods
end   % classdef
