%% This script compute the bounding box and the set of robots to be 
% rearraged to obtain coverage local based on a user defined radius around
% the failed robot. 

% update the active robot list
Rob_active(fail_rob_label) = 0;
fail_rob_pos = Rob_active_pos(indx,:);
Rob_active_lab(indx) = [];
Rob_active_pos(indx,:) = [];

% compute the bounding box coordinates
b_box = [max([fail_rob_pos(1)-radius_tune env_min_x])...
    min([fail_rob_pos(1)+radius_tune env_max_x]);
    max([fail_rob_pos(2)-radius_tune env_min_y])...
    min([fail_rob_pos(2)+radius_tune env_max_y])];
% length of bounding box along x
b_box_x = b_box(1,2) -b_box(1,1);
% length of bounding box along y
b_box_y = b_box(2,2) -b_box(2,1);


% robots inside and outside the region
fail_rob_nbh = zeros(length(Rob_active_lab),1);
com_fail_rob_nbh = zeros(length(Rob_active_lab),1);
com_fail_rob_nbh_pos = zeros(length(Rob_active_lab),2);
p = 1;
q = 1;
% find the robots in the sensing area
for i = 1:length(Rob_active_lab)
    if norm(fail_rob_pos(1) - Rob_active_pos(i,1)) <= b_box_x && ...
            norm(fail_rob_pos(2) - Rob_active_pos(i,2)) <= b_box_y
        fail_rob_nbh(p) = Rob_active_lab(i);
        p = p + 1;
    else
        com_fail_rob_nbh(q) = Rob_active_lab(i);
        com_fail_rob_nbh_pos(q,:) = Rob_active_pos(i,:);
        q = q + 1;
    end
end
fail_rob_nbh(p:end) = [];
com_fail_rob_nbh(q:end) = [];
com_fail_rob_nbh_pos(q:end,:) = [];

