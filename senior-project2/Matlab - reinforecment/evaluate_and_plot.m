function evaluate_and_plot(ppoFile, sacFile)
% EVALUATE_AND_PLOT  Load trained agent(s), evaluate, and produce all 8 plots
% required by the Phase III report.
%
% USAGE
%   evaluate_and_plot('saved_agents_ppo/ppo_final.mat')
%   evaluate_and_plot('saved_agents_ppo/ppo_final.mat', ...
%                     'saved_agents_sac/sac_final.mat')
%
% PLOTS GENERATED
%   1  Training reward curve (episode + 50-ep moving average)
%   2  Reward component breakdown per step (one evaluation episode)
%   3  Joint angle trajectories (one evaluation episode)
%   4  Joint velocity trajectories (smoothness verification)
%   5  End-effector 3D trajectory (pick -> carry -> place)
%   6  Evaluation histogram (100 episodes)
%   7  CoM trajectory vs support polygon boundary
%   8  PPO vs SAC learning curve comparison

if nargin < 1 || isempty(ppoFile)
    ppoFile = fullfile('saved_agents_ppo', 'ppo_final.mat');
end
if nargin < 2
    sacFile = '';
end

%% ── Load PPO ────────────────────────────────────────────────────────────
if ~isfile(ppoFile)
    error('evaluate_and_plot: PPO file not found:\n  %s\nRun train_agent(''ppo'') first.', ppoFile);
end
fprintf('Loading PPO agent: %s\n', ppoFile);
ppoData  = load(ppoFile, 'agent', 'trainingStats');
ppoAgent = ppoData.agent;
ppoStats = ppoData.trainingStats;

%% ── Create a fresh evaluation environment ───────────────────────────────
env = MobileManipulatorEnv();

% ════════════════════════════════════════════════════════════════
%  PLOT 1:  Training Reward Curve (PPO)
% ════════════════════════════════════════════════════════════════
fig1 = figure('Name','Plot 1 – PPO Training Reward Curve', ...
    'Position',[30 620 860 340], 'Color','w');
epR   = ppoStats.EpisodeReward;
avgR  = movmean(epR, 50);
nEp   = numel(epR);
plot(1:nEp, epR,  'Color',[0.75 0.75 0.90], 'LineWidth',0.8); hold on;
plot(1:nEp, avgR, 'Color',[0.12 0.37 0.65], 'LineWidth',2.5);
yline(250,'--r','LineWidth',1.5,'Label','Convergence target (250)', ...
    'LabelHorizontalAlignment','left');
xlabel('Episode'); ylabel('Total Episode Reward');
title('PPO Training — Episode Reward','FontWeight','bold');
legend('Per-episode reward','50-episode moving avg','Location','southeast');
grid on; box on;
saveas(fig1,'plot1_training_reward.png');
fprintf('Plot 1 saved: plot1_training_reward.png\n');

% ════════════════════════════════════════════════════════════════
%  Run ONE detailed evaluation episode  (Plots 2-5, 7)
% ════════════════════════════════════════════════════════════════
fprintf('Running detailed evaluation episode...\n');
[detLog, totalR, success] = runDetailedEpisode(ppoAgent, env);

nStep  = size(detLog.q, 2);
t_vec  = (0:nStep-1) * env.Ts;
fprintf('  Steps: %d  |  Total reward: %.1f  |  Success: %d\n', ...
    nStep, totalR, success);

% ════════════════════════════════════════════════════════════════
%  PLOT 2:  Reward Components per Step
% ════════════════════════════════════════════════════════════════
fig2 = figure('Name','Plot 2 – Reward Components', ...
    'Position',[30 230 900 360], 'Color','w');
cmap = lines(8);
compLabels = {'r_{reach}','r_{orient}','r_{grasp}','r_{carry}', ...
              'r_{place}','r_{jlim}','r_{tip}','r_{smooth}'};
hold on;
for k = 1:8
    plot(t_vec, detLog.rc(k,:), 'Color',cmap(k,:), 'LineWidth',1.5, ...
        'DisplayName',compLabels{k});
end
xlabel('Time (s)'); ylabel('Reward value');
title('Reward Component Breakdown — Evaluation Episode','FontWeight','bold');
legend('Location','best','NumColumns',2);
grid on; box on;
saveas(fig2,'plot2_reward_components.png');
fprintf('Plot 2 saved: plot2_reward_components.png\n');

% ════════════════════════════════════════════════════════════════
%  PLOT 3:  Joint Angle Trajectories
% ════════════════════════════════════════════════════════════════
fig3 = figure('Name','Plot 3 – Joint Angles', ...
    'Position',[900 620 820 340], 'Color','w');
jc = [0.22 0.48 0.87; 0.85 0.33 0.10; 0.47 0.67 0.19];
hold on;
for j = 1:3
    plot(t_vec, rad2deg(detLog.q(j,:)), 'Color',jc(j,:), 'LineWidth',2, ...
        'DisplayName', sprintf('q_%d',j));
end
% Add joint limit lines
jLim_deg = rad2deg(env.JointLimits);
for j = 1:2   % only J1 limits for clarity
    yline(jLim_deg(j,1), ':k', 'LineWidth',0.8);
    yline(jLim_deg(j,2), ':k', 'LineWidth',0.8);
end
xlabel('Time (s)'); ylabel('Joint Angle (deg)');
title('Joint Angle Trajectories — Trained PPO Policy','FontWeight','bold');
legend({'q_1 (Shoulder)','q_2 (Elbow)','q_3 (Wrist)'},'Location','best');
grid on; box on;
saveas(fig3,'plot3_joint_angles.png');
fprintf('Plot 3 saved: plot3_joint_angles.png\n');

% ════════════════════════════════════════════════════════════════
%  PLOT 4:  Joint Velocity Trajectories  (smoothness check)
% ════════════════════════════════════════════════════════════════
fig4 = figure('Name','Plot 4 – Joint Velocities', ...
    'Position',[900 230 820 340], 'Color','w');
hold on;
for j = 1:3
    plot(t_vec, detLog.qdot(j,:), 'Color',jc(j,:), 'LineWidth',1.8, ...
        'DisplayName', sprintf('\\dot{q}_%d',j));
end
yline( env.MaxJointVel(1),'--k','v_{max}','LineWidth',0.8);
yline(-env.MaxJointVel(1),'--k','v_{min}','LineWidth',0.8);
xlabel('Time (s)'); ylabel('Joint Velocity (rad/s)');
title('Joint Velocity Trajectories — r_{smooth} reduces jerk','FontWeight','bold');
legend('Location','best');
grid on; box on;
saveas(fig4,'plot4_joint_velocities.png');
fprintf('Plot 4 saved: plot4_joint_velocities.png\n');

% ════════════════════════════════════════════════════════════════
%  PLOT 5:  End-Effector 3D Trajectory
% ════════════════════════════════════════════════════════════════
fig5 = figure('Name','Plot 5 – EE 3D Trajectory', ...
    'Position',[30 230 660 500], 'Color','w');
ee = detLog.ee_w;   % world-frame EE  3 x T
plot3(ee(1,:), ee(2,:), ee(3,:), 'b-', 'LineWidth',2); hold on;
scatter3(ee(1,1),   ee(2,1),   ee(3,1),   80, 'ko','filled');   % start
scatter3(detLog.pickPos(1), detLog.pickPos(2), detLog.pickPos(3), ...
         120, 'r^','filled');                                     % pick zone
scatter3(env.GoalZone(1), env.GoalZone(2), env.GoalZone(3), ...
         120, 'gs','filled');                                     % goal
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('End-Effector 3D Trajectory','FontWeight','bold');
legend('EE path','Start','Pick zone','Goal zone','Location','best');
grid on; view(35,25);
saveas(fig5,'plot5_ee_trajectory.png');
fprintf('Plot 5 saved: plot5_ee_trajectory.png\n');

% ════════════════════════════════════════════════════════════════
%  PLOT 6:  Evaluation Histogram (100 episodes)
% ════════════════════════════════════════════════════════════════
fprintf('Running 100 evaluation episodes for histogram...\n');
nEval     = 100;
allRewards = zeros(nEval,1);
nSuccess   = 0;
for ep = 1:nEval
    [~, epR, epSucc] = runDetailedEpisode(ppoAgent, env);
    allRewards(ep) = epR;
    nSuccess = nSuccess + epSucc;
    if mod(ep,25)==0, fprintf('  Episode %d/%d\n', ep, nEval); end
end
sRate = 100 * nSuccess / nEval;

fig6 = figure('Name','Plot 6 – Evaluation Histogram', ...
    'Position',[900 230 700 380], 'Color','w');
histogram(allRewards, 20, 'FaceColor',[0.22 0.48 0.87], 'EdgeColor','w');
xline(mean(allRewards),'--r','LineWidth',2, ...
    'Label',sprintf('Mean=%.0f',mean(allRewards)), ...
    'LabelHorizontalAlignment','right');
xlabel('Total Episode Reward'); ylabel('Count');
title(sprintf('PPO Evaluation — %d episodes  |  Success rate: %.0f%%', ...
    nEval, sRate),'FontWeight','bold');
grid on; box on;
fprintf('Success rate: %.0f%%  |  Mean: %.1f +/- %.1f\n', ...
    sRate, mean(allRewards), std(allRewards));
saveas(fig6,'plot6_eval_histogram.png');
fprintf('Plot 6 saved: plot6_eval_histogram.png\n');

% ════════════════════════════════════════════════════════════════
%  PLOT 7:  CoM Trajectory vs Support Polygon
% ════════════════════════════════════════════════════════════════
fig7 = figure('Name','Plot 7 – CoM Stability', ...
    'Position',[30 230 660 480], 'Color','w');
com = detLog.com;   % 3 x T, in shoulder frame

% Support polygon projected on arm XZ plane (X=forward, Z=up)
hl = env.HalfBaseLen;
sp_x = [-hl*1.8, hl*1.8, hl*1.8, -hl*1.8, -hl*1.8];
sp_z = [-env.ShoulderHeight, -env.ShoulderHeight, ...
         env.ShoulderHeight*0.5, env.ShoulderHeight*0.5, -env.ShoulderHeight];
fill(sp_x, sp_z, [0.90 0.97 0.90], 'EdgeColor',[0.2 0.7 0.3], ...
    'LineWidth',2, 'DisplayName','Support polygon'); hold on;
plot(com(1,:), com(3,:), 'r-', 'LineWidth',1.8, 'DisplayName','CoM (X-Z)');
scatter(com(1,1), com(3,1), 70, 'ko','filled','DisplayName','CoM start');
xline( hl*1.8,'--r','LineWidth',1,'Label','Tip limit');
xline(-hl*1.8,'--r','LineWidth',1);
xlabel('X_{CoM} [m] (forward)'); ylabel('Z_{CoM} [m] (vertical)');
title('Projected Centre of Mass vs Support Polygon','FontWeight','bold');
legend('Location','best'); grid on; box on;
saveas(fig7,'plot7_com_stability.png');
fprintf('Plot 7 saved: plot7_com_stability.png\n');

% ════════════════════════════════════════════════════════════════
%  PLOT 8:  PPO vs SAC Learning Curves
% ════════════════════════════════════════════════════════════════
fig8 = figure('Name','Plot 8 – PPO vs SAC Comparison', ...
    'Position',[30 620 860 360], 'Color','w');

% PPO curve
ppoAvg = movmean(ppoStats.EpisodeReward, 50);
plot(1:numel(ppoStats.EpisodeReward), ppoStats.EpisodeReward, ...
    'Color',[0.75 0.75 0.90],'LineWidth',0.6); hold on;
plot(1:numel(ppoAvg), ppoAvg, ...
    'Color',[0.12 0.37 0.65],'LineWidth',2.5,'DisplayName','PPO (50-ep avg)');

% SAC curve (if file provided)
hasSAC = ~isempty(sacFile) && isfile(sacFile);
if hasSAC
    sacData  = load(sacFile,'trainingStats');
    sacStats = sacData.trainingStats;
    sacAvg   = movmean(sacStats.EpisodeReward, 50);
    plot(1:numel(sacStats.EpisodeReward), sacStats.EpisodeReward, ...
        'Color',[1.0 0.80 0.75],'LineWidth',0.6);
    plot(1:numel(sacAvg), sacAvg, ...
        'Color',[0.85 0.25 0.10],'LineWidth',2.5,'DisplayName','SAC (50-ep avg)');
else
    fprintf('[INFO] No SAC file found. Plot 8 shows PPO only.\n');
    fprintf('       Run train_agent(''sac'') to enable the comparison.\n');
end

yline(250,'--k','LineWidth',1.2,'Label','Target avg reward','DisplayName','Target');
xlabel('Episode'); ylabel('50-Episode Average Reward');
title('PPO vs SAC — Learning Curve Comparison','FontWeight','bold');
legend('Location','southeast'); grid on; box on;
saveas(fig8,'plot8_ppo_vs_sac.png');
fprintf('Plot 8 saved: plot8_ppo_vs_sac.png\n');

fprintf('\n=== All 8 plots generated and saved. ===\n');
end   % main function


% ════════════════════════════════════════════════════════════════
%  LOCAL HELPER: run one episode and collect detailed log
% ════════════════════════════════════════════════════════════════
function [log, totalReward, success] = runDetailedEpisode(agent, env)
% Runs a single deterministic episode and returns logged trajectories.
%
% OUTPUTS
%   log.q        3 x T  joint angles (rad)
%   log.qdot     3 x T  joint velocities (rad/s)
%   log.ee_w     3 x T  EE world position (m)
%   log.com      3 x T  CoM in shoulder frame (m)
%   log.rc       8 x T  reward components
%   log.pickPos  3 x 1  pick zone world position
%   totalReward  scalar
%   success      logical

[obs, ~]    = reset(env);
pickPos     = env.ObjectPos;   % save randomised pick position before any grasp
isDone      = false;
totalReward = 0;

q_log    = [];  qdot_log = [];  ee_log = [];
com_log  = [];  rc_log   = [];

while ~isDone
    % Log BEFORE stepping (captures initial state too)
    q_log    = [q_log,    env.q];                     %#ok<AGROW>
    qdot_log = [qdot_log, env.qdot];                  %#ok<AGROW>
    ee_log   = [ee_log,   env.getEEWorldFrame()];      %#ok<AGROW>
    com_log  = [com_log,  env.getCoM()];               %#ok<AGROW>
    rc_log   = [rc_log,   [env.Rc_reach;  env.Rc_orient; ...
                            env.Rc_grasp; env.Rc_carry;  ...
                            env.Rc_place; env.Rc_jlim;   ...
                            env.Rc_tip;   env.Rc_smooth]]; %#ok<AGROW>

    % Get greedy action
    actionCell  = getAction(agent, {obs});
    action      = actionCell{1};

    [obs, reward, isDone, ~] = step(env, action);
    totalReward = totalReward + reward;
end

% Append final state
q_log    = [q_log,    env.q];
qdot_log = [qdot_log, env.qdot];
ee_log   = [ee_log,   env.getEEWorldFrame()];
com_log  = [com_log,  env.getCoM()];
rc_log   = [rc_log,   [env.Rc_reach;  env.Rc_orient; ...
                        env.Rc_grasp; env.Rc_carry;  ...
                        env.Rc_place; env.Rc_jlim;   ...
                        env.Rc_tip;   env.Rc_smooth]];

success = env.ObjectGrasped && ...
          (norm(env.ObjectPos - env.GoalZone) < env.PlaceTol);

log.q       = q_log;
log.qdot    = qdot_log;
log.ee_w    = ee_log;
log.com     = com_log;
log.rc      = rc_log;
log.pickPos = pickPos;
end
