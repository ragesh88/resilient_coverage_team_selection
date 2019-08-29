function [info,rob_sel] = prob1_MILP(costs,areas, R_val, beta, alpha, d_area)
% This function takes the inputs of the MILP problem for problem one and
% splits out the binary solution for the problem 
%INPUTs
% costs : the vector containing the cost of every robot
% areas : the vector containing the area covered by each robot
% R_val : the vector containing the reliablility function values
% beta : max budget
% alpha : max failure probability
% d_area : desired area coverage

%   Detailed explanation goes here
% the function solves a mixed integer linear programing problem using the
% solver Mosek

Tot_agents = length(costs);

prob.c = ones(1, Tot_agents);
prob.a = [costs'; -log(1-R_val'); areas'];
prob.blc = [-inf -log(alpha) d_area];
prob.buc = [beta inf inf];
prob.blx = zeros(1, Tot_agents);
prob.bux = ones(1, Tot_agents);

prob.ints.sub = 1:Tot_agents;

% parameter list 
param.MSK_IPAR_LOG = 0;
param.MSK_IPAR_NUM_THREADS =  10;

% Optimize the problem.
[~,res] = mosekopt('maximize',prob,param);
try
% Display the optimal solution.
% res.sol.int
info = res.rcodestr;
rob_sel = res.sol.int.xx;
catch
fprintf('MSKERROR: Could not get solution')
end


end
