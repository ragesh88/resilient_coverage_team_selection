% function: greedy algorithm
function [gre_set, h, prob_pos] = gre_place(Rob_labels, R_x, delta,...
    b_box, Rob_sen_rads, pre_set, pre_set_pos)
    % the function computes the position of the robots in greedy fashion.
    % The parameters of the function are:
    % INPUTS 
    % Rob_labels : the labels of the robots selected
    % R_x : probability of occurance of events
    % b_box : [x_min x_max; y_min y_max] the coordinates of the
    % bounding box. 
    % Rob_sen_rads : the sensing radius of the robots
    % pre_set : the pre selected set of robots
    % pre_set_pos : the positions of the preselected robots
    
%     global N env_size R_x delta
%     global com_range
    env_x_range = b_box(1,1):delta:b_box(1,2);
    env_y_range = b_box(2,1):delta:b_box(2,2);
    
    env_x_size = length(env_x_range);
    env_y_size = length(env_y_range);
    
    % number of selected robots
    N = length(Rob_labels);
    gre_set = []; % define the set selected by greedy
    
    
    for k = 1 : N % greedy needs N rounds
        % compute a 100*100 table of h_values for a sensor placed at each (i,j)
        h_sensor = zeros(env_x_size,env_y_size);
        % and the h_value at each point with each placement of all sensors {x,y}
        h_pos_all_sensor = cell(env_x_size,env_y_size);

        % coordination (i,j)
        for i = 1:env_x_size
            for j = 1:env_y_size
               % check if this position is taken
               if ~isempty([pre_set_pos; gre_set]) ...
                       && any(ismembertol([pre_set_pos; gre_set],...
                       [env_x_range(i),env_y_range(j)],'ByRows',true))
                   continue;
               end
%                 [k,i,j]
                % if (i,j) is not inside obstacle
                if ~is_in_obstacle(env_x_range(i),env_y_range(j))
%                         && 
%                     is_neigbor([env_x_range(i),env_y_range(j)],...
%                         [pre_set_pos; gre_set], ...
%                         com_range*ones(size([pre_set_pos; gre_set],1),1))
                    % compute h function by considering sensors already placed
                    [h_sensor(i,j), h_pos_all_sensor{i,j}] = h_compute(...
                        [pre_set_pos; gre_set],...
                        [env_x_range(i),env_y_range(j)],...
                        b_box, delta, R_x,...
                        Rob_sen_rads([pre_set; Rob_labels(1:k)]));
                else % (i,j) is inside obstacle
                    h_sensor(i,j) = -inf; % make sure it will never be chosen
                end
            end
        end
        % at each round, find the maximal h and the pos to place sensors
        max_h = max(h_sensor(:));
        [ind_i,ind_j] = find(h_sensor==max_h,1);

        % store the pos [x_max,y_max] into greedy set
        gre_set = [gre_set; [env_x_range(ind_i), env_y_range(ind_j)]]; 
        h_pos_temp = h_pos_all_sensor{ind_i,ind_j};
    end
    % the final coverage by greedy approach
    h = max_h; 
    prob_pos = h_pos_temp/R_x;
end