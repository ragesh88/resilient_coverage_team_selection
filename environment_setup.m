% function: simulate an environment
% 50*50 square with three rectangular obstacles
function environment_setup()

    global env_size R_x delta...
    obs_1x_min obs_1x_max obs_1y_min obs_1y_max...
    obs_2x_min obs_2x_max obs_2y_min obs_2y_max...
    obs_3x_min obs_3x_max obs_3y_min obs_3y_max...
    obs_1x obs_1y len_1x len_1y...
    obs_2x obs_2y len_2x len_2y...
    obs_3x obs_3y len_3x len_3y...
    obs_1 obs_2 obs_3...
    
    env_size = 50; % environment size
    
    R_x = 1; % unifrom density 
    delta = 2; % discretization parameter

    % generate three rectangular obstacles
    % 1st obstace
    obs_1x = 10;
    obs_1y = 10;
    len_1x = 5;
    len_1y = 30;
    obs_1x_min = obs_1x;
    obs_1x_max = obs_1x + len_1x;
    obs_1y_min = obs_1y;
    obs_1y_max = obs_1y + len_1y;
    obs_1 = polyshape([obs_1x_min obs_1x_max obs_1x_max obs_1x_min], ...
        [obs_1y_min obs_1y_min obs_1y_max obs_1y_max]);

    % 2nd obstacle
    obs_2x =20;
    obs_2y=5;
    len_2x = 30;
    len_2y = 5; 
    obs_2x_min = obs_2x;
    obs_2x_max = obs_2x + len_2x;
    obs_2y_min = obs_2y;
    obs_2y_max = obs_2y + len_2y;
    obs_2 = polyshape([obs_2x_min obs_2x_max obs_2x_max obs_2x_min], ...
        [obs_2y_min obs_2y_min obs_2y_max obs_2y_max]);

    % 3rd obstacle
    obs_3x =35;
    obs_3y=25;
    len_3x = 10;
    len_3y = 10; 
    obs_3x_min = obs_3x;
    obs_3x_max = obs_3x + len_3x;
    obs_3y_min = obs_3y;
    obs_3y_max = obs_3y + len_3y;
    obs_3 = polyshape([obs_3x_min obs_3x_max obs_3x_max obs_3x_min], ...
        [obs_3y_min obs_3y_min obs_3y_max obs_3y_max]);
end