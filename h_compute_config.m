function [h_sensor] = h_compute_config(pos, b_box, delta, R_x,...
    Rob_sen_rads)
% the function computes the coverage value of the whole domain given the
% positions of the robots
% INPUTS
% pos : position of the robots
% b_box : the area for the coverage is to be checked
% delta : discretization of the dimensions
% R_x : probability of event occurance 
    
    global lambda
    
     env_x_range = b_box(1,1):delta:b_box(1,2);
    env_y_range = b_box(2,1):delta:b_box(2,2);
    
    env_x_size = length(env_x_range);
    env_y_size = length(env_y_range);
    
    % compute h for each (x,y)
    h_pos= zeros(env_x_size, env_y_size);
    
    % update sensor set
    set_plus = pos;
    
    for i = 1: env_x_size
        for j = 1: env_y_size
            % if (x,y) is inside the obstacle
            if is_in_obstacle(env_x_range(i),env_y_range(j)) 
                h_pos(i,j) = 0;
            else
                % check all the previous sensors selected
                prob_temp = 1; 
                for k = 1 : length(set_plus(:,1))
                    % if (x,y) is inside set(k,:)'s visibility range
                    if is_in_visibility([env_x_range(i),env_y_range(j)]...
                            , set_plus(k,:), Rob_sen_rads(k))
                        prob_temp = prob_temp *...
                            (1 - exp(-lambda *...
                            norm([env_x_range(i),env_y_range(j)]-set_plus(k,:))));
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