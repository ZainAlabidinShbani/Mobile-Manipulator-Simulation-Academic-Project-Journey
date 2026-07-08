function agent = setup_sac_agent(env)
% SETUP_SAC_AGENT  Build and return a SAC agent for MobileManipulatorEnv.
%
% Architecture (report Section 7 — SAC branch):
%   Actor   14 -> FC(256,ReLU) -> FC(256,ReLU) -> FC(5,linear)
%   Q-net1  [obs(14) | act(5)] -> concat(19) -> FC(256,ReLU) -> FC(256,ReLU) -> 1
%   Q-net2  identical copy (double-Q trick reduces overestimation bias)
%
% R2023b API:
%   rlContinuousGaussianActor, rlQValueFunction, rlSACAgent,
%   rlSACAgentOptions, rlOptimizerOptions, concatenationLayer, dlnetwork

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
nObs    = obsInfo.Dimension(1);   % 14
nAct    = actInfo.Dimension(1);   % 5

% ================================================================
%  SAC ACTOR  (larger network than PPO; benefits from diverse replay)
% ================================================================
sacActorLG = layerGraph();
sacActorLG = addLayers(sacActorLG, [
    featureInputLayer(nObs, 'Normalization','none', 'Name','sac_obs')
    fullyConnectedLayer(256, 'Name','sac_afc1', 'WeightsInitializer','he')
    reluLayer(               'Name','sac_arel1')
    fullyConnectedLayer(256, 'Name','sac_afc2', 'WeightsInitializer','he')
    reluLayer(               'Name','sac_arel2')
]);

% Mean head (linear output)
sacActorLG = addLayers(sacActorLG, [
    fullyConnectedLayer(nAct, 'Name','sac_amean', 'WeightsInitializer','he')
]);

% Std head (positive output)
sacActorLG = addLayers(sacActorLG, [
    fullyConnectedLayer(nAct, 'Name','sac_astd', 'WeightsInitializer','he')
    softplusLayer('Name','sac_astd_sp')
]);

sacActorLG = connectLayers(sacActorLG, 'sac_arel2', 'sac_amean');
sacActorLG = connectLayers(sacActorLG, 'sac_arel2', 'sac_astd');

sacActorNet = dlnetwork(sacActorLG);

sacActor = rlContinuousGaussianActor(sacActorNet, obsInfo, actInfo, ...
    'ObservationInputNames', 'sac_obs', ...
    'ActionMeanOutputNames', 'sac_amean', ...
    'ActionStandardDeviationOutputNames', 'sac_astd_sp');

% ================================================================
%  SAC Q-NETWORKS  (two identical networks for double-Q trick)
%  Each takes [obs; action] concatenated as a 2-input layerGraph.
% ================================================================
qNet1 = buildQNet(nObs, nAct, '1');   % local helper (see below)
qNet2 = buildQNet(nObs, nAct, '2');

sacCritic1 = rlQValueFunction(qNet1, obsInfo, actInfo, ...
    'ObservationInputNames', 'obs', ...
    'ActionInputNames',      'act');

sacCritic2 = rlQValueFunction(qNet2, obsInfo, actInfo, ...
    'ObservationInputNames', 'obs', ...
    'ActionInputNames',      'act');

% ================================================================
%  SAC AGENT OPTIONS
% ================================================================
agentOpts = rlSACAgentOptions;
agentOpts.SampleTime             = env.Ts;   % 0.05 s
agentOpts.DiscountFactor         = 0.99;
agentOpts.TargetSmoothFactor     = 0.005;    % tau for soft target update
agentOpts.ExperienceBufferLength = 50000;    % replay buffer size
agentOpts.MiniBatchSize          = 256;      % large batches for off-policy
agentOpts.NumWarmStartSteps      = 1000;     % random actions before training starts

% Entropy weight: auto-tuned by default (target entropy = -numActions)
% To disable auto-tuning and use a fixed weight, uncomment:
% agentOpts.EntropyWeightOptions.TargetEntropy = -nAct;

agentOpts.ActorOptimizerOptions  = rlOptimizerOptions( ...
    'Algorithm','adam', 'LearnRate',3e-4, 'GradientThreshold',5.0);
agentOpts.CriticOptimizerOptions = rlOptimizerOptions( ...
    'Algorithm','adam', 'LearnRate',3e-4, 'GradientThreshold',5.0);

% ================================================================
%  CREATE AGENT
% ================================================================
agent = rlSACAgent(sacActor, [sacCritic1, sacCritic2], agentOpts);

fprintf('[SAC] Agent created  |  Obs:%d  Act:%d  Hidden:256x256(ReLU)  Double-Q\n', nObs, nAct);
end


% ----------------------------------------------------------------
function net = buildQNet(nObs, nAct, suffix)
% BUILDQNET  Two-input Q-network: [obs(14) | act(5)] -> Q(s,a).
% Uses concatenationLayer so obs and act enter as separate inputs,
% which is required by rlQValueFunction in R2023b.
%
% suffix ('1' or '2') ensures unique layer names for the two critics.

s = suffix;   % shorthand

lg = layerGraph();

% --- Input branches ---
lg = addLayers(lg, featureInputLayer(nObs, 'Normalization','none', 'Name','obs'));
lg = addLayers(lg, featureInputLayer(nAct, 'Normalization','none', 'Name','act'));

% --- Merge + shared trunk ---
% concatenationLayer(dim, numInputs): dim=1 concatenates along feature axis
lg = addLayers(lg, concatenationLayer(1, 2, 'Name',['cat' s]));

lg = addLayers(lg, [
    fullyConnectedLayer(256, 'Name',['q_fc1'  s], 'WeightsInitializer','he')
    reluLayer(               'Name',['q_rel1' s])
    fullyConnectedLayer(256, 'Name',['q_fc2'  s], 'WeightsInitializer','he')
    reluLayer(               'Name',['q_rel2' s])
    fullyConnectedLayer(1,   'Name',['q_out'  s], 'WeightsInitializer','he')
]);

% --- Connections ---
lg = connectLayers(lg, 'obs',       ['cat' s '/in1']);
lg = connectLayers(lg, 'act',       ['cat' s '/in2']);
lg = connectLayers(lg, ['cat' s],   ['q_fc1'  s]);

net = dlnetwork(lg);
end
