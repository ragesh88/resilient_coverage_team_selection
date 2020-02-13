%% This script test whether the matlab functions written for experiments
%% would work as planned 
clc
clearvars
close all

A_n = 10;

exp_parameters;

% compute the initial set of coordinates 
[Rob_active_pos, b_box] = exp_init_coord(A_n);

set_gre = Rob_active_pos;

Rob_active_lab = 1:A_n;
Rob_sel_labels = Rob_active_lab;
% plot the positions 

[~, h_pos] = h_compute_config(set_gre, b_box, delta, R_x,...
    Rob_sen_rads(Rob_sel_labels));
plots(set_gre, b_box, delta, h_pos);
hold on
% plot the robot labels 
for i = 1:length(Rob_active_lab)
    text(set_gre(i,1),set_gre(i,2), num2str(Rob_active_lab(i)));
end

%% failure simulation
Rob_active = ones(A_n,1);
for i_1 = 1:A_n
    % do this untill all robots fail
    % randomly select a node to fail
    indx = floor(rand(1)*sum(Rob_active));
    if indx == 0
        indx = 1;
    end
    fail_rob_label = Rob_active_lab(indx);
    fail_rob_pos = Rob_active_pos(indx,:);
    
    % compute the bounding box coordinates
    b_box = [max([fail_rob_pos(1)-radius_tune env_min_x])...
        min([fail_rob_pos(1)+radius_tune env_max_x]);
        max([fail_rob_pos(2)-radius_tune env_min_y])...
        min([fail_rob_pos(2)+radius_tune env_max_y])];
    [set_gre,Rob_active_lab] = exp_reconfig(A_n, ...
        radius_tune,...
        indx, Rob_active_lab, Rob_active_pos);
    
    Rob_active_lab(indx) = [];
    Rob_active_pos(indx,:) = [];
    
    % plot the new set of coordinates
    figure
    rectangle('Position',[ b_box(1,1) b_box(2,1) b_box(1,2)-b_box(1,1) b_box(2,2)-b_box(2,1)])
    hold on
    plot(fail_rob_pos(1), fail_rob_pos(2),'b*')
    
    t_box = [env_x(1) env_x(end)
        env_y(1) env_y(end)];
    % compute the new coverage values
    [~, h_pos] = h_compute_config(set_gre, t_box, delta, R_x,...
        Rob_sen_rads(Rob_sel_labels));
    % environment
    figure,
    plots(set_gre, t_box, delta, h_pos);
    hold on 

    for i = 1:length(Rob_active_lab)
        text(set_gre(i,1),set_gre(i,2), num2str(Rob_active_lab(i)));
    end
    answer=questdlg('Would you like to continue?', ...
	'Yes', ...
	'No');
    if (strlength(answer) == 2)
        close all
        break;
    else
        close all
    end
end