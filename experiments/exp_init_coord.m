% 

function [Rob_active_pos, b_box] = exp_init_coord(A_n)
%the function computes the initial coordinates for the robot experiment
%   Detailed explanation goes here
% A_n
% initialize the system with the parameters
% simulation parameters

% the vector indicating the available robots
Rob_pool = ones(A_n,1);


exp_parameters;

% add paths from the previous folder
addpath('..');

% compute the coordinate of the robots

% solve the MILP to select the robots
% [info, Rob_sel] = prob1_MILP(Rob_costs,Rob_areas,Rob_vals,budget,...
%     alpha,domain_area);

% select all robots

Rob_sel = Rob_pool;

% the set containing the selected robot labels
Rob_sel_labels = find(Rob_sel);


% bounding box of the domain
b_box = [env_x(1) env_x(end)
    env_y(1) env_y(end)];


% solve the greedy algorithm to place the robots
[set_gre, ~, ~] = gre_place(Rob_sel_labels, R_x, delta,...
    b_box, Rob_sen_rads,[],[]);

% update the list of available robots
% Rob_pool = Rob_pool - Rob_sel;
% update the list of active robots
% Rob_active = Rob_sel;
% labels of the active robots
% Rob_active_lab = find(Rob_active);
% positions of active robots
Rob_active_pos = set_gre;

end
