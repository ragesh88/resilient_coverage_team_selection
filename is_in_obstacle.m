% function: to determine if the pos is in the obstacle
function in_obs = is_in_obstacle(x,y)
    global obs_1x_min obs_1x_max obs_1y_min obs_1y_max...
    obs_2x_min obs_2x_max obs_2y_min obs_2y_max...
    obs_3x_min obs_3x_max obs_3y_min obs_3y_max
    
    % inside obstacle 1
    if x>=obs_1x_min && x<=obs_1x_max && y>=obs_1y_min && y<=obs_1y_max
        in_obs = true; 
    % inside obstacle 2
    elseif x>=obs_2x_min && x<=obs_2x_max && y>=obs_2y_min && y<=obs_2y_max
        in_obs = true; 
    % inside obstacle 3    
    elseif x>=obs_3x_min && x<=obs_3x_max && y>=obs_3y_min && y<=obs_3y_max
        in_obs = true; 
    else
        in_obs = false; 
    end
end