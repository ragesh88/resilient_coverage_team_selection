% The script to generate simulation outputs for a generated set of failures
clc
clearvars
close all

% simulation parameters
% number of robots in the selection pool
A_n = 50;
% the vector containing the values of the robots that 
Rob_pool = 1:A_n;

% realiability parameters
% generate mean failure time for all robots
MTTF_mean = 420;
MTTFs = normrnd(MTTF_mean, 0.1*MTTF_mean,[A_n 1]);
% max time of the mission
T_max = max(MTTFs);
% time span of the mission
T = [0 T_max];
% lambda zero in the reliability model
l0s = 1./(10*MTTFs);
% k in the reliability model
k = 0.0001 * l0s;
% required reliability
alpha = 0.3;


% Cost parameters
% the cost of each robot
max_cost = 50;
Rob_costs = max_cost*MTTFs/max(MTTFs);
budget = 500;

% environment parameters
env_size = 50; % environment size
R_x = 1; % unifrom density 
delta = 2; % discretization parameter
domain_area = env_size^2;

% area covered by each robot
max_area = 200;
Rob_areas = max_area * MTTFs/max(MTTFs);

% compute the reliability value for all
Rob_vals = zeros(A_n,1);
for i = 1:A_n
    Rob_vals(i) = reliability(T,l0s(i),k(i));
end
l_R_vals = -log(1-Rob_vals);
l_alpha = -log(alpha);

[info, Rob_sel] = prob1_MILP(Rob_costs,Rob_areas,Rob_vals,budget,alpha,domain_area);

% set the outputs paths
output_path = '/media/ragesh/Disk1/data/resilient_coverage/';
% find the contents in the output directory
files = dir(output_path);
% Get a logical vector that tells which is a directory.
dirFlags = [files.isdir] & ~strcmp({files.name},'.')...
    & ~strcmp({files.name},'..');
% Extract only those that are directories.
subFolders = files(dirFlags);
% compute the trail number
trail_no = length(subFolders) + 1;


% write the data to files in appropriate folders