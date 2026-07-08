%% main_run.m
% ================================================================
% MASTER SCRIPT — Mobile Manipulator RL Controller (Phase III)
% ================================================================
% Prerequisites
%   MATLAB R2023b
%   Reinforcement Learning Toolbox
%   Deep Learning Toolbox
%
% USAGE — Run sections individually or the whole script:
%
%   main_run          % runs full pipeline: validate → train PPO → evaluate
%
% To also train SAC and get Plot 8 comparison:
%   stats_ppo = train_agent('ppo');
%   stats_sac = train_agent('sac', 'MaxEpisodes', 500);
%   evaluate_and_plot('saved_agents_ppo/ppo_final.mat', ...
%                     'saved_agents_sac/sac_final.mat');
%
% For a 5-episode quick smoke test (no saving):
%   train_agent('ppo','MaxEpisodes',5,'ShowPlot',false)
% ================================================================

clear; clc; close all;
rng(42, 'twister');   % reproducible randomness

% ── Check required toolboxes ──────────────────────────────────────
required = {'Reinforcement Learning Toolbox', 'Deep Learning Toolbox'};
installed = {ver().Name};
missing = setdiff(required, installed);
if ~isempty(missing)
    warning('The following toolboxes may be missing:\n  %s', strjoin(missing,'\n  '));
end

% ── Add this folder to path ───────────────────────────────────────
addpath(fileparts(mfilename('fullpath')));

fprintf('=================================================\n');
fprintf(' Mobile Manipulator RL Controller — Phase III\n');
fprintf('=================================================\n\n');

%% STEP 1: Validate environment
fprintf('--- Step 1: Environment validation ---\n');
env_test = MobileManipulatorEnv();
try
    validateEnvironment(env_test);
    fprintf('[PASS] validateEnvironment passed.\n\n');
catch ME
    warning('[WARN] validateEnvironment reported: %s\n', ME.message);
end
clear env_test;

%% STEP 2: Train PPO (primary algorithm)
fprintf('--- Step 2: Train PPO ---\n');
ppoFile = fullfile('saved_agents_ppo','ppo_final.mat');
if isfile(ppoFile)
    fprintf('[SKIP] Found existing PPO agent: %s\n', ppoFile);
    fprintf('       Delete it or rename to retrain.\n\n');
else
    statsPPO = train_agent('ppo', ...
        'MaxEpisodes', 1000, ...
        'StopValue',   250,  ...
        'ShowPlot',    true);
    fprintf('\n[PPO] Mean reward (last 50): %.1f\n\n', ...
        mean(statsPPO.EpisodeReward(max(1,end-49):end)));
end

%% STEP 3: (Optional) Train SAC for comparison plot
% Uncomment to enable SAC training (~500 episodes)
%{
fprintf('--- Step 3: Train SAC ---\n');
sacFile = fullfile('saved_agents_sac','sac_final.mat');
if ~isfile(sacFile)
    statsSAC = train_agent('sac', ...
        'MaxEpisodes', 500, ...
        'StopValue',   250, ...
        'ShowPlot',    true);
end
%}
sacFile = '';   % set to SAC path above if SAC was trained

%% STEP 4: Evaluate and generate all 8 report plots
fprintf('--- Step 4: Evaluate + generate plots ---\n');
evaluate_and_plot(ppoFile, sacFile);

fprintf('\n=== main_run complete. All plots saved to current directory. ===\n');
