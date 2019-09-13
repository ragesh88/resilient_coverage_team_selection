% The script to generate simulation outputs for a generated set of failures
clc
clearvars
close all

% global variables
global lambda


% simulation parameters
% number of robots in the selection pool
A_n = 50;
% the vector indicating the available robots
Rob_pool = ones(A_n,1);

% tuning parameter
omega = 1;
coverage_thres = 120;

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
env_size = 30; % environment size
R_x = 1; % unifrom density 
delta = 2; % discretization parameter
domain_area = env_size^2;
env_x = 0:delta:env_size;
env_y = 0:delta:env_size;

% area covered by each robot
max_area = 200;
Rob_areas = max_area * MTTFs/max(MTTFs);
% sensing radius of the robots
Rob_sen_rads = sqrt(Rob_areas/pi);
% sensing parameter 
lambda = 0.1; % sensing decay parameter 

% compute the reliability value for all
Rob_vals = zeros(A_n,1);
for i = 1:A_n
    Rob_vals(i) = reliability(T,l0s(i),k(i));
end
l_R_vals = -log(1-Rob_vals);
l_alpha = -log(alpha);

% solve the MILP to select the robots
[info, Rob_sel] = prob1_MILP(Rob_costs,Rob_areas,Rob_vals,budget,...
    alpha,domain_area);

% the set containing the selected robot labels
Rob_sel_labels = find(Rob_sel);

% bounding box of the domain
b_box = [env_x(1) env_x(end)
    env_y(1) env_y(end)];

% solve the greedy algorithm to place the robots
[set_gre, h_gre, prob_pos_gre] = gre_place(Rob_sel_labels, R_x, delta,...
    b_box, Rob_sen_rads,[],[]);

% update the list of available robots
Rob_pool = Rob_pool - Rob_sel;
% update the list of active robots
Rob_active = Rob_sel;
% labels of the active robots
Rob_active_lab = find(Rob_active);

% construct the graph from the robots 
% adjacency matrix
adj = zeros(length(Rob_sel_labels));
nodenames = cell(length(Rob_sel_labels),1);
for i = 1:length(Rob_sel_labels)
    nodenames{i} = num2str(Rob_sel_labels(i));
end

% construct the adjacent matrix
for i = 1:length(Rob_sel_labels)
    for j = i+1:length(Rob_sel_labels)
        % compute the distance between the robots 
        dist = norm(set_gre(i,:) - set_gre(j,:));
        % communication radius sum
        sum_com = Rob_sen_rads(Rob_sel_labels(i)) + ...
            Rob_sen_rads(Rob_sel_labels(i));
        if dist <= sum_com
            adj(i,j) = dist;
            adj(j,i) = dist;
        end
    end
end
C_graph = graph(adj,nodenames);

plots(set_gre, prob_pos_gre, b_box, delta, adj,Rob_sen_rads(Rob_sel_labels));

%% PHASE II Failure simulation
% randomly select a node to fail
indx = floor(rand(1)*sum(Rob_active));
if indx == 0
    indx = 1;
end
fail_rob_label = Rob_active_lab(indx);

% find the nodes upto the omega+1 level



%% Writing data to file
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