%% This script generates the data to study change in coverage with respect 
% to change in radius of region.

clc
clearvars
close all

% global variables
global lambda com_range


% simulation parameters
% number of robots in the selection pool
A_n = 50;
% the vector indicating the available robots
Rob_pool = ones(A_n,1);

% environment parameters
env_size = 30; % environment size
env_min_x = 0;
env_min_y = 0;
env_max_x = 30;
env_max_y = 30;
R_x = 1; % unifrom density 
delta = 2; % discretization parameter
domain_area = env_size^2;
env_x = env_min_x:delta:env_max_x;
env_y = env_min_y:delta:env_max_y;

% tuning parameter
omega = 2; % hoops
% radius_tune = 10; % radius to consider for tuning
radius_tune_range = 5:5:env_size; % radius to consider for tuning
coverage_thres = 190;

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



% area covered by each robot
max_area = 200;
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
[info, Rob_sel] = prob1_MILP(Rob_costs,Rob_areas,Rob_vals,budget,...
    alpha,domain_area);

% the set containing the selected robot labels
Rob_sel_labels = find(Rob_sel);

% bounding box of the domain
b_box = [env_x(1) env_x(end)
    env_y(1) env_y(end)];

% solve the greedy algorithm to place the robots
[set_gre, h_gre, ~] = gre_place(Rob_sel_labels, R_x, delta,...
    b_box, Rob_sen_rads,[],[]);

base_coverage = h_gre;

% update the list of available robots
Rob_pool = Rob_pool - Rob_sel;
% update the list of active robots
Rob_active = Rob_sel;
% labels of the active robots
Rob_active_lab = find(Rob_active);
% positions of active robots
Rob_active_pos = set_gre;

%% failure simulation

% pick a random time for the failure event
rand_t = T(1) + (T(2)-T(1))*rand(1);
% evalute the reliability values of the robots at this time
R_vals_t = zeros(A_n,1);
for i = 1:A_n
    R_vals_t(i) = reliability([rand_t T_max],l0s(i),k(i));
end
% the reliabilty values of the active robots
R_vals_act = R_vals_t(Rob_active==1);
% select a failed robot through roulette wheel selection
indx = RouletteWheelSelection(R_vals_act);
fail_rob_label = Rob_active_lab(indx);
% total area after robot failure
% tot_area = tot_area - Rob_areas(fail_rob_label);
% % total area lost
% lost_area = lost_area + Rob_areas(fail_rob_label);
% rearrage based on omega tuning hoops tuning
% omega_tune;
% rearrange based on radius parameter
% iterate over various radius parameter
coverage_out = zeros(length(radius_tune_range),1);
for i = 1:length(radius_tune_range)
    radius_tune = radius_tune_range(i);
    rad_tune
    if isempty(fail_rob_nbh)
        continue;
    end
    % rearrage the robots to this local area
    % solve the greedy algorithm to place the robots
    [fail_rob_nbh_pos, ~, prob_pos_gre] = gre_place(fail_rob_nbh, R_x, delta,...
        b_box, Rob_sen_rads, com_fail_rob_nbh, com_fail_rob_nbh_pos);
    
    % rearrange the active robot set to match the ones in the coordinate order
    set_gre = [com_fail_rob_nbh_pos; fail_rob_nbh_pos];
    Rob_active_lab = [com_fail_rob_nbh; fail_rob_nbh];
    
    t_box = [env_x(1) env_x(end)
        env_y(1) env_y(end)];
    
    % the new coverage value computed
    h_gre_1 =  h_compute_config(set_gre, t_box, delta, R_x, Rob_sen_rads(Rob_active_lab));
    coverage_out(i) = h_gre_1;
end