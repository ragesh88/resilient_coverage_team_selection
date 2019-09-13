%% This script compute the bounding box and the set of robots to be 
% rearraged to obtain coverage local based on the number of hoops to be
% reached.

% find nodes upto the omega level
adj_omega = adj_u^omega;

% find robots at omega hops from the robot
fail_rob_nbh_indx = find(adj_omega(indx,:)>0);
fail_rob_nbh = Rob_active_lab(fail_rob_nbh_indx);

% find the complement set of fail_rob_nbhs in the active robot list 
com_fail_rob_nbh_indx = find(adj_omega(indx,:)==0);
com_fail_rob_nbh = Rob_active_lab(com_fail_rob_nbh_indx);

% update the active robot list
Rob_active(fail_rob_label) = 0;
Rob_active_lab(indx) = [];
Rob_active_pos(indx,:) = [];

% update the bounding box where the robots failed robot neighbor has to
% reconfigure
% find the minimum reachable coordinates in the failed robots 
min_coords_x = Rob_active_pos(:,1) - Rob_sen_rads(Rob_active_lab);
min_coords_x = max([min_coords_x'; env_min_x*ones(1,length(min_coords_x))])';
min_coords_y = Rob_active_pos(:,2) - Rob_sen_rads(Rob_active_lab);
min_coords_y = max([min_coords_y'; env_min_y*ones(1,length(min_coords_y))])';
% find the maximum reachable coordinates of the failed robots
max_coords_x = Rob_active_pos(:,1) + Rob_sen_rads(Rob_active_lab);
max_coords_x = min([max_coords_x'; env_max_x*ones(1,length(max_coords_x))])';
max_coords_y = Rob_active_pos(:,2) + Rob_sen_rads(Rob_active_lab);
max_coords_y = min([max_coords_y'; env_max_y*ones(1,length(max_coords_y))])';

% compute the bounding box coordinates
b_box = [min(min_coords_x) min(min_coords_y);
    max(max_coords_x) max(max_coords_y)];