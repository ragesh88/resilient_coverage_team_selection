% this script for the MILP problem
clc
clearvars
MTTF_mean = 420;
A_n = 50;
MTTFs = normrnd(MTTF_mean, 0.1*MTTF_mean,[A_n 1]);
T_max = max(MTTFs);
T = [0 T_max];
l0s = 1./(10*MTTFs);
k = 0.0001 * l0s;
% the cost of each robot
max_cost = 50;
costs = max_cost*MTTFs/max(MTTFs);
% defining the domain
domain_area = 1000;
% area covered by each robot
max_area = 200;
areas = max_area * MTTFs/max(MTTFs);

alpha = 0.3;
budget = 500;
R_vals = zeros(A_n,1);
for i = 1:A_n
    R_vals(i) = reliability(T,l0s(i),k(i));
end
l_R_vals = -log(1-R_vals);
l_alpha = -log(alpha);
[info, sel] = prob1_MILP(costs,areas,R_vals,budget,alpha,domain_area);

% check viability of the solution
if strcmp(info, 'MSK_RES_OK') && any(sel)
    fprintf('Total area covered by the selected group %f \n', sum(areas(sel == 1)));
    fprintf('domain ares %f \n', domain_area);
    fprintf('team failure probability %f \n', prod(1-R_vals(sel==1)));
    fprintf('acceptable failure probability %f \n', alpha);
    fprintf('team budget %f \n', sum(costs(sel==1)));
    fprintf('Max budget %f \n', budget);
end