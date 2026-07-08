function agent = setup_ppo_agent(env)
% SETUP_PPO_AGENT  Build and return a PPO agent for MobileManipulatorEnv.
%
% Network architecture (report Section 7):
%   Actor  14 -> FC(64,tanh) -> FC(64,tanh) -> FC(5,linear)   [action mean]
%   Critic 14 -> FC(64,tanh) -> FC(64,tanh) -> FC(1,linear)   [V(s)]
%
% Report errors fixed here:
%   - Actor LR:  3e4  -> 3e-4  (missing negative sign)
%   - Critic LR: 1e3  -> 1e-3  (missing negative sign)
%   - rlPPOAgentOptions() syntax corrected (report had missing comma)
%   - No final tanh on actor output (MATLAB scales to action bounds)
%
% R2023b API used:
%   rlContinuousGaussianActor, rlValueFunction, rlPPOAgent,
%   rlPPOAgentOptions, rlOptimizerOptions, dlnetwork, layerGraph

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
nObs    = obsInfo.Dimension(1);   % 14
nAct    = actInfo.Dimension(1);   % 5

% ================================================================
%  ACTOR NETWORK  (stochastic Gaussian policy: outputs action mean)
%  Log-std is a separate set of learnable parameters (state-independent)
%  MATLAB R2023b handles this automatically inside rlContinuousGaussianActor
% ================================================================
% Shared trunk -> two heads (mean, std)
actorLG = layerGraph();
actorLG = addLayers(actorLG, [
    featureInputLayer(nObs, 'Normalization','none', 'Name','ppo_obs')
    fullyConnectedLayer(64,  'Name','ppo_afc1', 'WeightsInitializer','he')
    tanhLayer(               'Name','ppo_atau1')
    fullyConnectedLayer(64,  'Name','ppo_afc2', 'WeightsInitializer','he')
    tanhLayer(               'Name','ppo_atau2')
]);

% Mean head (linear output)
actorLG = addLayers(actorLG, [
    fullyConnectedLayer(nAct, 'Name','ppo_amean', 'WeightsInitializer','he')
]);

% Std head (positive output)
actorLG = addLayers(actorLG, [
    fullyConnectedLayer(nAct, 'Name','ppo_astd', 'WeightsInitializer','he')
    softplusLayer('Name','ppo_astd_sp')
]);

actorLG = connectLayers(actorLG, 'ppo_atau2', 'ppo_amean');
actorLG = connectLayers(actorLG, 'ppo_atau2', 'ppo_astd');

actorNet = dlnetwork(actorLG);

actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo, ...
    'ObservationInputNames', 'ppo_obs', ...
    'ActionMeanOutputNames', 'ppo_amean', ...
    'ActionStandardDeviationOutputNames', 'ppo_astd_sp');

% ================================================================
%  CRITIC NETWORK  (state-value function V(s))
% ================================================================
criticLayers = [
    featureInputLayer(nObs, 'Normalization','none', 'Name','ppo_cobs')
    fullyConnectedLayer(64,  'Name','ppo_cfc1', 'WeightsInitializer','he')
    tanhLayer(               'Name','ppo_ctanh1')
    fullyConnectedLayer(64,  'Name','ppo_cfc2', 'WeightsInitializer','he')
    tanhLayer(               'Name','ppo_ctanh2')
    fullyConnectedLayer(1,   'Name','ppo_cval',  'WeightsInitializer','he')
];

criticNet = dlnetwork(layerGraph(criticLayers));

critic = rlValueFunction(criticNet, obsInfo, ...
    'ObservationInputNames', 'ppo_cobs');

% ================================================================
%  PPO AGENT OPTIONS
% ================================================================
agentOpts = rlPPOAgentOptions(...
    'SampleTime',           env.Ts,  ...  % 0.05 s (20 Hz)
    'DiscountFactor',       0.99,    ...  % far-sighted: long-horizon task
    'GAEFactor',            0.95,    ...  % Generalised Advantage Estimation lambda
    'ClipFactor',           0.20,    ...  % PPO clipping epsilon
    'EntropyLossWeight',    0.01,    ...  % entropy regularisation coefficient
    'MiniBatchSize',        64,      ...
    'NumEpoch',             10,      ...  % gradient passes per experience horizon
    'ExperienceHorizon',    512);         % steps collected before each update

% Optimiser  (report had typos: 3e4 and 1e3 — corrected to 3e-4 and 1e-3)
agentOpts.ActorOptimizerOptions  = rlOptimizerOptions( ...
    'Algorithm','adam', 'LearnRate',3e-4, 'GradientThreshold',1.0);
agentOpts.CriticOptimizerOptions = rlOptimizerOptions( ...
    'Algorithm','adam', 'LearnRate',1e-3, 'GradientThreshold',1.0);

% ================================================================
%  CREATE AGENT
% ================================================================
agent = rlPPOAgent(actor, critic, agentOpts);

fprintf('[PPO] Agent created  |  Obs:%d  Act:%d  Hidden:64x64(tanh)\n', nObs, nAct);
end
