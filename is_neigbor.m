% function to check if the current sensor is the neighbor of one of previous sensors
% two main conditions. 1st, communication range. 2nd, inside visibility
function is_nei = is_neigbor(pos, set, com_range)
% the function checks of the selected position is in the neighborhood for
% some existing robot. 
% pos : the position to be tested
% set : postions of existing of robots
% com_range : comunication range of robots in same order as set

is_nei = false;
% set is empty
if isempty(set)
    is_nei = true;
else % set has some sensors chosen
    for i = 1 : length(set(:,1))
        if is_in_visibility(pos, set(i,:), com_range(i))
            if norm(pos - set(i,:))<= com_range(i) % assume com_range <= sensing range
                is_nei = true;
                break;
            end
        end
    end
end
end