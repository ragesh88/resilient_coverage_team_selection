%% this script saves the data into a folder with the name 
% coverage_thresh Trail_#trail number'
clc
clearvars
% global gamma
% number of trails to be done
no_of_trails = 10;
gamma = 1.2;
coverage_thres = gamma*100;
% output folder path
out_fldr_pth = '/media/ragesh/Disk1/data/resilient_coverage/robots_vs_r/gamma/';
out_fldr_pth = [out_fldr_pth num2str(coverage_thres) '/'];
if ~exist(out_fldr_pth, 'dir')
    mkdir(out_fldr_pth);
end
% % find the contents in the output directory
files = dir(out_fldr_pth);
% Get a logical vector that tells which is a directory.
dirFlags = [files.isdir] & ~strcmp({files.name},'.')...
    & ~strcmp({files.name},'..');
% Extract only those that are directories.
subFolders = files(dirFlags);
% compute the starting trail number
trail_no_srt = length(subFolders) + 1;

for trail = trail_no_srt:no_of_trails
    fprintf('\nstarting trail: %d',trail);
    new_robots_r_data_gamma;
    % make the directory to saving the data
    new_out_folder = [out_fldr_pth 'trail_' num2str(trail)];
    mkdir(new_out_folder);
    % create a README file in the folder
    readmefileid = fopen([new_out_folder '/README.md'], 'w');
    fprintf(readmefileid, '### README\n');
    fprintf(readmefileid,['## This folder contains the outputs of a simulation'...
        'performed by a team of robots. The each robot in the team fails'...
        'randomly. The team is rearranged based on a bounding box. Here we'...
        'generate data to understand the number of robots requested\n\n']);
    fprintf(readmefileid,'# Inputs for the simulation\n');
    fprintf(readmefileid,'-Number of robots : %d\n', A_n);
    fprintf(readmefileid,'-The minimum x coordinate of environment : %f\n',...
        env_min_x);
    fprintf(readmefileid,'-The minimum y coordinate of environment : %f\n',...
        env_min_y);
    fprintf(readmefileid,'-The maximum x coordinate of environment : %f\n',...
        env_max_x);
    fprintf(readmefileid,'-The maximum coordinate of environment : %f\n',...
        env_max_y);
    fprintf(readmefileid,'-The discretization along x axis : %f\n', delta);
    fprintf(readmefileid,'-The discretization along y axis : %f\n', delta);
    fprintf(readmefileid,'-The coverage threshold : %f\n', delta);
    fprintf(readmefileid,'-The tuning radius range minimum : %f\n',...
        radius_tune_range(1));
    fprintf(readmefileid,'-The tuning radius range maximum : %f\n',...
        radius_tune_range(end));
    fprintf(readmefileid,'-The tuning radius range increment : %f\n',...
        radius_tune_range(2)-radius_tune_range(1));
    fprintf(readmefileid,'-Design failure probability (alpha) : %f\n', alpha);
    fprintf(readmefileid,'-Design budget : %f\n', budget);
    fprintf(readmefileid,'-Design area : %f\n', desired_area);
    fprintf(readmefileid,'-Sensing decay factor : %f\n', lambda);   
    fprintf(readmefileid,'-Base coverage : %f\n', base_coverage);  
    fprintf(readmefileid,'-Coverage parameter: %f\n', gamma);  
    fclose(readmefileid);
    
    % write the required data to csv files
    csvwrite([new_out_folder '/radius_tune_range.csv'],  radius_tune_range);
    csvwrite([new_out_folder  '/robots.csv'], prob3_robots);
    fprintf('finished trail: %d',trail);
end
