% The script to generate experiment script outputs for a generated set of failures
% in this script we exclude the case where the robots are added to the team
clc
clearvars
close all

% add paths from the previous folder
addpath('..')  

% global variables
global lambda com_range


% simulation parameters
% number of robots in the selection pool
A_n = 10;
% the vector indicating the available robots
Rob_pool = ones(A_n,1);

% tuning parameter
omega = 2; % hoops
radius_tune = .5; % radius to consider for tuning
coverage_thres = 0;

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
env_size = 4; % environment size
env_min_x = -2;
env_min_y = -2;
env_max_x = 2;
env_max_y = 2;
R_x = 1; % unifrom density 
delta = 0.25; % discretization parameter
domain_area = env_size^2;
env_x = env_min_x:delta:env_max_x;
env_y = env_min_y:delta:env_max_y;

% area covered by each robot
max_area = 5;
Rob_areas = max_area *MTTFs/max(MTTFs);
% sensing radius of the robots
Rob_sen_rads = sqrt(Rob_areas/pi);
% sensing parameter 
lambda = 0.1; % sensing decay parameter 
% communication range
com_range = max(Rob_sen_rads);

% compute the reliability value for all
Rob_vals = zeros(A_n,1);
for i = 1:A_n
    Rob_vals(i) = reliability(T,l0s(i),k(i));
end
l_R_vals = -log(1-Rob_vals);
l_alpha = -log(alpha);

% solve the MILP to select the robots
% [info, Rob_sel] = prob1_MILP(Rob_costs,Rob_areas,Rob_vals,budget,...
%     alpha,domain_area);

% the set containing the selected robot labels
Rob_sel_labels = find(Rob_pool);


% bounding box of the domain
b_box = [env_x(1) env_x(end)
    env_y(1) env_y(end)];


% solve the greedy algorithm to place the robots
[set_gre, h_gre, ~] = gre_place(Rob_sel_labels, R_x, delta,...
    b_box, Rob_sen_rads,[],[]);

% update the list of available robots
% Rob_pool = Rob_pool - Rob_sel;
% update the list of active robots
Rob_active = Rob_pool;
% labels of the active robots
Rob_active_lab = find(Rob_active);
% positions of active robots
Rob_active_pos = set_gre;


[~, h_pos] = h_compute_config(set_gre, b_box, delta, R_x,...
    Rob_sen_rads(Rob_sel_labels));
plots(set_gre, b_box, delta, h_pos);
hold on
% plot the robot labels 
for i = 1:length(Rob_active_lab)
    text(set_gre(i,1),set_gre(i,2), num2str(Rob_active_lab(i)));
end

%% PHASE II Failure simulation
tot_area = sum(Rob_areas(Rob_active_lab));
lost_area = 0;
% generate the time for next failure
% randomly select a node to fail
indx = floor(rand(1)*sum(Rob_active));
if indx == 0
    indx = 1;
end
fail_rob_label = Rob_active_lab(indx);
% total area after robot failure
tot_area = tot_area - Rob_areas(fail_rob_label);
% total area lost
lost_area = lost_area + Rob_areas(fail_rob_label);
% rearrage based on omega tuning hoops tuning
% omega_tune;
% rearrange based on radius parameter
rad_tune

% rearrage the robots to this local area
% solve the greedy algorithm to place the robots
[fail_rob_nbh_pos, ~, prob_pos_gre] = gre_place(fail_rob_nbh, R_x, delta,...
    b_box, Rob_sen_rads, com_fail_rob_nbh, com_fail_rob_nbh_pos);

% rearrange the active robot set to match the ones in the coordinate order
set_gre = [com_fail_rob_nbh_pos; fail_rob_nbh_pos];
Rob_active_lab = [com_fail_rob_nbh; fail_rob_nbh];

% construct the graph from the robots 
% adjacency matrix
% adj = zeros(length(Rob_active_lab));
% nodenames = cell(length(Rob_active_lab),1);
% for i = 1:length(Rob_active_lab)
%     nodenames{i} = num2str(Rob_active_lab(i));
% end
% 
% % construct the adjacent matrix
% for i = 1:length(Rob_active_lab)
%     for j = i+1:length(Rob_active_lab)
%         % compute the distance between the robots 
%         dist = norm(set_gre(i,:) - set_gre(j,:));
%         % communication radius sum
%         max_com = com_range;
%         if dist <= max_com
%             adj(i,j) = dist;
%             adj(j,i) = dist;            
%         end
%     end
% end
% % unweighted adjacency matrix
% adj_u = double(adj>0);

t_box = [env_x(1) env_x(end)
    env_y(1) env_y(end)];

rectangle('Position',[ b_box(1,1) b_box(2,1) b_box(1,2)-b_box(1,1) b_box(2,2)-b_box(2,1)])
plot(fail_rob_pos(1), fail_rob_pos(2),'b*')
% compute the new coverage values
[~, h_pos] = h_compute_config(set_gre, t_box, delta, R_x,...
    Rob_sen_rads(Rob_sel_labels));
figure,
plots(set_gre, t_box, delta, h_pos);
hold on 
for i = 1:length(Rob_active_lab)
    text(set_gre(i,1),set_gre(i,2), num2str(Rob_active_lab(i)));
end
rectangle('Position',[ b_box(1,1) b_box(2,1) b_box(1,2)-b_box(1,1) b_box(2,2)-b_box(2,1)])

[h_gre_1, ~ ] =  h_compute_config(set_gre, t_box, delta, R_x, Rob_sen_rads(Rob_active_lab));

% invoke the intermediate selection MILP to get new robot which 
prob3_request = 0;
while h_gre_1 < coverage_thres
    alpha1 = alpha/(prod(1-Rob_vals(Rob_active_lab)));
    [info, Rob_sel] = prob3_MILP(Rob_areas,Rob_vals,...
    alpha1,lost_area, Rob_pool);
    % update the list of available robots
    Rob_pool = Rob_pool - Rob_sel;
    % add the selected robots to list of failed robot neighbors
    fail_rob_nbh = [fail_rob_nbh; find(Rob_sel)];
    % recompute the coverage
    [fail_rob_nbh_pos, ~, prob_pos_gre] = gre_place(fail_rob_nbh, R_x, delta,...
    b_box, Rob_sen_rads, com_fail_rob_nbh, com_fail_rob_nbh_pos);
    % rearrange the active robot set to match the ones in the coordinate order
    set_gre = [com_fail_rob_nbh_pos; fail_rob_nbh_pos];
    Rob_active_lab = [com_fail_rob_nbh; fail_rob_nbh];
    h_gre_1 =  h_compute_config(set_gre, t_box, delta, R_x, Rob_sen_rads(Rob_active_lab));
    prob3_request = prob3_request + 1;
end

% compute the new coverage values
[~, h_pos] = h_compute_config(set_gre, t_box, delta, R_x,...
    Rob_sen_rads(Rob_sel_labels));

figure
plots(set_gre, t_box, delta, h_pos);
hold on 
for i = 1:length(Rob_active_lab)
    text(set_gre(i,1),set_gre(i,2), num2str(Rob_active_lab(i)));
end
rectangle('Position',[ b_box(1,1) b_box(2,1) b_box(1,2)-b_box(1,1) b_box(2,2)-b_box(2,1)])

% %% Writing data to file
% % set the outputs paths
% output_path = '/media/ragesh/Disk1/data/resilient_coverage/';
% % find the contents in the output directory
% files = dir(output_path);
% % Get a logical vector that tells which is a directory.
% dirFlags = [files.isdir] & ~strcmp({files.name},'.')...
%     & ~strcmp({files.name},'..');
% % Extract only those that are directories.
% subFolders = files(dirFlags);
% % compute the trail number
% trail_no = length(subFolders) + 1;
% 
% 
% % write the data to files in appropriate folders