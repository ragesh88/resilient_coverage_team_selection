function [Rob_active_pos, b_box, data_pth] = exp_init_traj(A_n, ...
    start, data_pth, fail_rob_nbh)
%exp_init_traj This function generates the initial coordinates
% trajectories for the robot teams 
%   Detailed explanation goes here
% A_n : number of robots
% start : the start coordinates of the robots
% compute the initial set of coordinates 
% data_pth : data storage path
[Rob_active_pos, b_box] = exp_init_coord(A_n);

aug_Rob_active_pos = [Rob_active_pos 1.5*ones(size(Rob_active_pos,1),1)];

% compute the trail number
files = dir(data_pth);
% Get a logical vector that tells which is a directory.
dirFlags = [files.isdir] & ~strcmp({files.name},'.')...
    & ~strcmp({files.name},'..');
folder_name = num2str(length(find(dirFlags))+1);
data_pth = [data_pth folder_name];
mkdir(data_pth);


% generate trajectory for the robots for initial configuration
    if (isempty(fail_rob_nbh))        
        curdir = pwd; % take note of current folder
        cd('../../crazyswarm-planning'); % change to that folder
        
        traj_resilent_cvrge_exp(data_pth, 1:A_n, ...
            start, ...
            aug_Rob_active_pos, b_box)
        cd(curdir);
    end


end

