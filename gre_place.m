% function: greedy algorithm
function [gre_set, h, prob_pos] = gre_place()
    
    global N env_size R_x delta
    
    gre_set = []; % define the set selected by greedy
    
    for k = 1 : N % greedy needs N rounds
        % compute a 100*100 table of h_values for a sensor placed at each (i,j)
        h_sensor = zeros(env_size,env_size);
        % and the h_value at each point with each placement of all sensors {x,y}
        h_pos_all_sensor = cell(env_size,env_size);

        % coordination (i,j)
        for i = 1 : delta : env_size 
            for j = 1: delta : env_size
                [k,i,j]
                % if (i,j) is not inside obstacle
                if ~is_in_obstacle(i,j) && is_neigbor([i,j], gre_set)
                    % compute h function by considering sensors already placed
                    [h_sensor(i,j), h_pos_all_sensor{i,j}] = h_compute(gre_set, [i,j]);
                else % (i,j) is inside obstacle
                    h_sensor(i,j) = -inf; % make sure it will never be chosen
                end
            end
        end
        % at each round, find the maximal h and the pos to place sensors
        max_h = max(h_sensor(:));
        [x_max,y_max] = find(h_sensor==max_h,1);

        % store the pos [x_max,y_max] into greedy set
        gre_set = [gre_set; [x_max,y_max]]; 
        h_pos_temp = h_pos_all_sensor{x_max,y_max};
    end
    % the final coverage by greedy approach
    h = max_h; 
    prob_pos = h_pos_temp/R_x;
end