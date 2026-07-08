function trainingStats = train_agent(algorithm, varargin)
% TRAIN_AGENT  Train PPO or SAC on the MobileManipulatorEnv.
%
% USAGE
%   stats = train_agent('ppo')               % default: 1000 episodes
%   stats = train_agent('sac')
%   stats = train_agent('ppo','MaxEpisodes',500,'StopValue',220)
%
% OUTPUT
%   trainingStats  – rlTrainingStats object from the MATLAB RL Toolbox.
%                    Access episode rewards with trainingStats.EpisodeReward

if nargin < 1, algorithm = 'ppo'; end

%% Parse optional arguments
p = inputParser;
addParameter(p,'MaxEpisodes', 1000, @isnumeric);
addParameter(p,'StopValue',   250,  @isnumeric);
addParameter(p,'SaveDir',     '',   @ischar);
addParameter(p,'ShowPlot',    true, @islogical);
parse(p, varargin{:});

maxEp    = p.Results.MaxEpisodes;
stopVal  = p.Results.StopValue;
showPlot = p.Results.ShowPlot;

if isempty(p.Results.SaveDir)
    saveDir = fullfile(pwd, ['saved_agents_' lower(algorithm)]);
else
    saveDir = p.Results.SaveDir;
end

%% Create environment
env = MobileManipulatorEnv();

% Quick sanity check (optional – comment out to skip)
try
    validateEnvironment(env);
    fprintf('[%s] Environment validation passed.\n', upper(algorithm));
catch ME
    warning('[%s] validateEnvironment: %s', upper(algorithm), ME.message);
end

%% Create agent
switch lower(algorithm)
    case 'ppo'
        agent = setup_ppo_agent(env);
    case 'sac'
        agent = setup_sac_agent(env);
    otherwise
        error('train_agent: algorithm must be ''ppo'' or ''sac'', got ''%s''.', algorithm);
end

%% Training options
if ~exist(saveDir, 'dir'), mkdir(saveDir); end

plotOpt = 'none';
if showPlot, plotOpt = 'training-progress'; end

trainOpts = rlTrainingOptions(...
    'MaxEpisodes',                maxEp,       ...
    'MaxStepsPerEpisode',         200,          ...
    'ScoreAveragingWindowLength', 50,           ...
    'StopTrainingCriteria',       'AverageReward', ...
    'StopTrainingValue',          stopVal,      ...
    'SaveAgentCriteria',          'EpisodeReward', ...
    'SaveAgentValue',             200,          ...  % save when ep. reward > 200
    'SaveAgentDirectory',         saveDir,      ...
    'Verbose',                    true,         ...
    'Plots',                      plotOpt,      ...
    'UseParallel',                false);       % set true if Parallel Toolbox available

%% Train
fprintf('\n=== Training %s | maxEp=%d | stopAt=%.0f ===\n\n', ...
    upper(algorithm), maxEp, stopVal);

trainingStats = train(agent, env, trainOpts);

%% Save final agent
finalFile = fullfile(saveDir, [lower(algorithm) '_final.mat']);
save(finalFile, 'agent', 'trainingStats', 'algorithm');
fprintf('\n[%s] Training complete. Agent saved to:\n  %s\n', ...
    upper(algorithm), finalFile);
end
