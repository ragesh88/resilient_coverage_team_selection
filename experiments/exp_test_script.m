%% This script test whether the matlab functions written for experiments
%% would work as planned 
clc
clearvars
close all

A_n = 7;

exp_parameters;

% data path to store trajectories
data_pth = '/media/ragesh/Disk1/data/resilient_coverage/exp/';

% compute the trail number
files = dir(data_pth);
% Get a logical vector that tells which is a directory.
dirFlags = [files.isdir] & ~strcmp({files.name},'.')...
    & ~strcmp({files.name},'..');
folder_name = num2str(length(find(dirFlags))+1);
data_pth = [data_pth folder_name '/'];
mkdir(data_pth);

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


Rob_active = ones(A_n,1);

%% failure simulation
t_box = [env_x(1) env_x(end)
        env_y(1) env_y(end)];
    
for i_1 = 1:A_n-2

    % do this untill all robots fail
    % randomly select a node to fail
    indx = floor(rand(1)*sum(Rob_active));
    if indx == 0
        indx = 1;
    end
    
    
    figure
    rectangle('Position',[ t_box(1,1) t_box(2,1)...
        t_box(1,2)-t_box(1,1) t_box(2,2)-t_box(2,1)])
    hold on
    
    for i = 1:length(Rob_active_lab)
        text(set_gre(i,1),set_gre(i,2), num2str(Rob_active_lab(i)));
    end
    
    title('before failure');
    
    fail_rob_label = Rob_active_lab(indx);
    Rob_active(fail_rob_label) = 0;

    fail_rob_pos = Rob_active_pos(indx,:);
    
    % compute the bounding box coordinates
    b_box = [max([fail_rob_pos(1)-radius_tune env_min_x])...
        min([fail_rob_pos(1)+radius_tune env_max_x]);
        max([fail_rob_pos(2)-radius_tune env_min_y])...
        min([fail_rob_pos(2)+radius_tune env_max_y])];

    
    rectangle('Position',[ b_box(1,1) b_box(2,1)...
        b_box(1,2)-b_box(1,1) b_box(2,2)-b_box(2,1)])    
    
    for i = 1:length(Rob_active_lab)
        plot(set_gre(i,1),set_gre(i,2), 'rs');
    end
    
    
    [set_gre,com_fail_rob_nbh, fail_rob_nbh,b_box_1] = exp_reconfig(A_n, radius_tune,...
        indx, Rob_active_lab, Rob_active_pos);
    
    err = b_box(:) - b_box_1(:);
    
    assert(all(err == 0))
    
    % gather the previous positions of the robots in the failed 
    % neighbourhood
    fail_rob_nbh_pos_old = Rob_active_pos(...
        ismember(Rob_active_lab,fail_rob_nbh),:);
    
    Rob_active_lab = [com_fail_rob_nbh; fail_rob_nbh];
    
    fail_rob_nbh_pos_new = set_gre(end-length(fail_rob_nbh)+1:end,:);
    Rob_active_pos = set_gre;  
    
    
    
    
    % plot the new set of coordinates
    plot(fail_rob_pos(1), fail_rob_pos(2),'b*')       
    
    % generate trajectory for the robots in the failed neighbourhood
    if (length(fail_rob_nbh)>1)        
        curdir = pwd; % take note of current folder
        cd('../../crazyswarm-planning'); % change to that folder
        % padd the coordinates with extra dimensions
        aug_fail_rob_nbh_pos_old = [fail_rob_nbh_pos_old ...
            1.5*ones(length(fail_rob_nbh),1)];
        aug_fail_rob_nbh_pos_new = [fail_rob_nbh_pos_new ...
            1.5*ones(length(fail_rob_nbh),1)];
        traj_resilent_cvrge_exp(data_pth, fail_rob_nbh, ...
            aug_fail_rob_nbh_pos_old, ...
            aug_fail_rob_nbh_pos_new,...
            b_box)
        cd(curdir);
    end
    
   
    
    % gather the previous positions of the robots in the failed 
    % neighbourhood
    fail_rob_nbh_pos_old = Rob_active_pos(...
        ismember(Rob_active_lab,fail_rob_nbh),:);
    
    Rob_active_lab = [com_fail_rob_nbh; fail_rob_nbh];
    
    fail_rob_nbh_pos_new = set_gre(end-length(fail_rob_nbh)+1:end,:);
    Rob_active_pos = set_gre;  
    
    
    
    
    % plot the new set of coordinates
    plot(fail_rob_pos(1), fail_rob_pos(2),'b*')       
    
   
    

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

    title('after failure');
    

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

%% create a block diagonal matrix through Kronekar product
n = 5;
bl_sz = 2;
% create basis vectors
e=@(k,n) [zeros(k-1,1);1;zeros(n-k,1)];

% create blocks of the block diagonal matrix 
D = zeros(n*bl_sz,bl_sz);
for i = 1:n
    D((i-1)*bl_sz + 1: (i)*bl_sz,:) = round(100*rand(bl_sz));
end
Blk_D = zeros(n*bl_sz);
for i =1:n
    Blk_D = Blk_D + kron(e(i,n)*e(i,n)',eye(bl_sz))*D;
end