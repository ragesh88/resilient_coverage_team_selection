function [info,rob_sel] = prob3_MILP(areas, R_val, alpha, d_area, R_pool)
% This function takes the inputs of the MILP problem for problem three and
% splits out the binary solution for the problem 
%INPUTs
% areas : the vector containing the area covered by each robot
% R_val : the vector containing the reliablility function values
% alpha : max failure probability
% d_area : desired area coverage
% R_pool : vector indicating avaliability of robots

%   Detailed explanation goes here
% the function solves a mixed integer linear programing problem using the
% solver Mosek

Tot_agents = length(areas);

prob.c = ones(1, Tot_agents);
prob.a = [-log(1-R_val'); areas'];
prob.blc = [-log(alpha) d_area];
prob.buc = [inf inf];
prob.blx = zeros(1, Tot_agents);
prob.bux = R_pool;

prob.ints.sub = 1:Tot_agents;

% parameter list 
param.MSK_IPAR_LOG = 0;
param.MSK_IPAR_NUM_THREADS =  10;

% Optimize the problem.
[~,res] = mosekopt('minimize',prob,param);
try
% Display the optimal solution.
% res.sol.int
info = res.rcodestr;
rob_sel = res.sol.int.xx;
catch
fprintf('MSKERROR: Could not get solution')
end


end
