% The parameter for the experiment

% global variables
global lambda com_range


addpath('..') 
% tuning parameter
omega = 2; % hoops
radius_tune = 2; % radius to consider for tuning
% coverage_thres = 0.20;

% realiability parameters
% generate mean failure time for all robots
MTTF_mean = 420;
MTTFs = MTTF_mean*ones(A_n,1);
% max time of the mission
T_max = max(MTTFs);
% time span of the mission
T = [0 T_max];
% lambda zero in the reliability model
l0s = 1./(10*MTTFs);
% k in the reliability model
k = 0.0001 * l0s;
% required reliability
alpha = 0.3;


% Cost parameters
% the cost of each robot
max_cost = 50;
Rob_costs = max_cost*MTTFs/max(MTTFs);
budget = 500;

% environment parameters
env_size = 4; % environment size
env_min_x = -2;
env_min_y = -2;
env_max_x = 2;
env_max_y = 2;
R_x = 1; % unifrom density 
delta = 0.25; % discretization parameter
domain_area = env_size^2;
env_x = env_min_x:delta:env_max_x;
env_y = env_min_y:delta:env_max_y;

% area covered by each robot
max_area = 5;
Rob_areas = max_area *MTTFs/max(MTTFs);
% sensing radius of the robots
Rob_sen_rads = sqrt(Rob_areas/pi);
% sensing parameter 
lambda = 0.1; % sensing decay parameter 
% communication range
com_range = max(Rob_sen_rads);

% compute the reliability value for all
Rob_vals = zeros(A_n,1);
for i = 1:A_n
    Rob_vals(i) = reliability(T,l0s(i),k(i));
end
l_R_vals = -log(1-Rob_vals);
l_alpha = -log(alpha);