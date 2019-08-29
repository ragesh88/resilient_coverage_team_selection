function [r] = reliability(T,l0, k)
%reliability : the function computes the realiability of the system given
%the system parameters l0, k for a given time interval [T0, T]
%   Detailed explanation goes here
% INPUTS
% T : the time interval for reliability evalution
% l0: initial rate of failure
% k : failure rate increase 
% OUTPUTS
% r : reliability of the system of the time period

arg = l0*(T(2)-T(1)) + (k/2)*(T(2)^3 - T(1)^3);
r = exp(-arg);
end

