function [set_gre,Rob_active_lab] = exp_reconfig(A_n, ...
    radius_tune,...
    indx, Rob_active_lab, Rob_active_pos)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
exp_parameters;
tot_area = sum(Rob_areas(Rob_active_lab));
lost_area = 0;
% env_min_x = env(1,1);
% env_min_y = env(2,1);
% env_max_x = env(1,2);
% env_max_y = env(2,2);
fail_rob_label = Rob_active_lab(indx);
% total area after robot failure
tot_area = tot_area - Rob_areas(fail_rob_label);
% total area lost
lost_area = lost_area + Rob_areas(fail_rob_label);
exp_rad_tune;

% local reconfiguration code
% solve the greedy algorithm to place the robots
[fail_rob_nbh_pos, ~, prob_pos_gre] = gre_place(fail_rob_nbh, R_x, delta,...
    b_box, Rob_sen_rads, com_fail_rob_nbh, com_fail_rob_nbh_pos);

% rearrange the active robot set to match the ones in the coordinate order
set_gre = [com_fail_rob_nbh_pos; fail_rob_nbh_pos];
Rob_active_lab = [com_fail_rob_nbh; fail_rob_nbh];


end

