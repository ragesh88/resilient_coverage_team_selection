function [] = strt_exp_scrpt()
%strt_exp_scrpt function ensure the planning scripts can be run
%   Detailed explanation goes here
addpath(genpath('../../crazyswarm-planning'))
cd('../../crazyswarm-planning/continuous')
mex_all;
cd('..')
end

