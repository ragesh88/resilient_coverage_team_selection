% function to check if the current sensor is the neighbor of one of previous sensors
% two main conditions. 1st, communication range. 2nd, inside visibility
function is_nei = is_neigbor(pos, set)
global com_range
is_nei = false;
% set is empty
if isempty(set)
    is_nei = true;
else % set has some sensors chosen
    for i = 1 : length(set(:,1))
        if is_in_visibility(pos, set(i,:))
            if norm(pos - set(i,:))<= com_range % assume com_range <= sensing range
                is_nei = true;
                break;
            end
        end
    end
end
end