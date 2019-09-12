% function: check if a pos (x,y) is inside a sensor's visibility region
% because of the blocking from obstacles
function is_in_vis = is_in_visibility(pos, sensor)
    
    global obs_1 obs_2 obs_3 sen_range

    is_in_vis = false;
    
    % to see if the line linking pos and sensor is inside the obstacles
    line = [pos; sensor];
    
    if norm(pos - sensor)<= sen_range
        % three obstacles & sensing range
        if isempty(intersect(obs_1,line))
            if isempty(intersect(obs_2,line))
               if isempty(intersect(obs_3,line))
                 is_in_vis = true;
               end
            end
        end
    end
    
end