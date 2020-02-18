function [Rob_active_lab, Rob_active_pos] = exp_inter_traj(A_n, radius_tune,...
        indx, Rob_active_lab, Rob_active_pos, data_pth)
%exp_inter_traj This function takes the values of the intermediate
%coordinates and generates the coordinates between them 
%   Detailed explanation goes here
% A_n : number of robots
% radius_tune : radius to select the neighbourhood
% indx : index of the failed robot
% Rob_active_lab : labels of the active robot plus the failed one
% Rob_active_pos : position of the active robots
% data_pth : folder to store generated trajectory

[set_gre,com_fail_rob_nbh, fail_rob_nbh] = exp_reconfig(A_n, radius_tune,...
        indx, Rob_active_lab, Rob_active_pos);

   % gather the previous positions of the robots in the failed 
    % neighbourhood
    fail_rob_nbh_pos_old = Rob_active_pos(...
        ismember(Rob_active_lab,fail_rob_nbh),:);
    
    Rob_active_lab = [com_fail_rob_nbh; fail_rob_nbh];
    
    fail_rob_nbh_pos_new = set_gre(end-length(fail_rob_nbh)+1:end,:);
    Rob_active_pos = set_gre;
  
 % generate trajectory for the robots in the failed neighbourhood
    if (~isempty(fail_rob_nbh))        
        curdir = pwd; % take note of current folder
        cd('../../crazyswarm-planning'); % change to that folder
        % padd the coordinates with extra dimensions
        aug_fail_rob_nbh_pos_old = [fail_rob_nbh_pos_old ...
            1.5*ones(length(fail_rob_nbh),1)];
        aug_fail_rob_nbh_pos_new = [fail_rob_nbh_pos_new ...
            1.5*ones(length(fail_rob_nbh),1)];
        traj_resilent_cvrge_exp(data_pth, fail_rob_nbh, ...
            aug_fail_rob_nbh_pos_old, ...
            aug_fail_rob_nbh_pos_new)
        cd(curdir);
    end

end

