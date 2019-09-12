% function: plot probability map, sensor placement and the environment
function plots(set, prob_pos)
    
    global env_size...
        obs_1x obs_1y len_1x len_1y...
        obs_2x obs_2y len_2x len_2y...
        obs_3x obs_3y len_3x len_3y
    
    
    % plot probability map
    prob_90 = [];
    prob_50 = [];
    for i = 1 : env_size
        for j = 1 : env_size
            if is_in_obstacle(i,j) == 0
                if prob_pos(i,j)>= 0.90
                    prob_90 = [prob_90; [i,j]];
                elseif prob_pos(i,j)>= 0.50 && prob_pos(i,j)<0.90
                    prob_50 = [prob_50; [i,j]];
                end
            end
        end
    end

    % plot probability map
    plot(prob_90(:,1), prob_90(:,2), 'bo'); hold on
    plot(prob_50(:,1), prob_50(:,2), 'go'); hold on

    % plot sensor positions
    plot(set(:,1), set(:,2), 'ro', 'MarkerFaceColor', 'r'); hold on
    % link sensors. They are connected

    % plot obstacles
    rectangle('Position',[obs_1x obs_1y len_1x len_1y], 'EdgeColor', 'g', 'FaceColor', 'g')
    rectangle('Position',[obs_2x obs_2y len_2x len_2y], 'EdgeColor', 'g', 'FaceColor', 'g')
    rectangle('Position',[obs_3x obs_3y len_3x len_3y], 'EdgeColor', 'g', 'FaceColor', 'g')
    % plot(obs_1, 'EdgeColor', 'g', 'FaceColor', 'g'); hold on 
    % plot(obs_2, 'EdgeColor', 'g', 'FaceColor', 'g'); hold on
    % plot(obs_3, 'EdgeColor', 'g', 'FaceColor', 'g'); hold on 
    
    % environment boundary
    axis([1 env_size 1 env_size])
    axis equal
    box on
end