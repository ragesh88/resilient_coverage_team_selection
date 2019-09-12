% function: compute the h_value at each point for each placement
function [h_sensor, h_pos] = h_compute(set, pos)
    
    global env_size lambda R_x delta
    
    % compute h for each (x,y)
    h_pos= zeros(env_size, env_size);
    
    % update sensor set
    set_plus = [set; pos];
    
    for i = 1 : delta : env_size
        for j = 1 : delta : env_size
            % if (x,y) is inside the obstacle
            if is_in_obstacle(i,j) 
                h_pos(i,j) = 0;
            else
                % check all the previous sensors selected
                prob_temp = 1; 
                for k = 1 : length(set_plus(:,1))
                    % if (x,y) is inside set(k,:)'s visibility range
                    if is_in_visibility([i,j], set_plus(k,:))
                        prob_temp = prob_temp * (1 - exp(-lambda * norm([i,j]-set_plus(k,:))));
                    end
                end
                % prob for each (x,y)
                prob = 1 - prob_temp;
                h_pos(i,j) = prob * R_x; 
            end
        end
    end
    % sum_up  h_pos
    h_sensor = sum(h_pos(:));
end