% function: plot probability map, sensor placement and the environment
function plots(set, b_box, delta, radius)
    
    global ...
        obs_1x obs_1y len_1x len_1y...
        obs_2x obs_2y len_2x len_2y...
        obs_3x obs_3y len_3x len_3y
    
    env_x_range = b_box(1,1):delta:b_box(1,2);
    env_y_range = b_box(2,1):delta:b_box(2,2);
    
    env_x_size = length(env_x_range);
    env_y_size = length(env_y_range);
    
    % plot probability map
%     prob_90 = [];
%     prob_50 = [];
%     for i = 1 : env_x_size
%         for j = 1 : env_y_size
%             if is_in_obstacle(i,j) == 0
%                 if prob_pos(i,j)>= 0.90
%                     prob_90 = [prob_90; [i,j]];
%                 elseif prob_pos(i,j)>= 0.50 && prob_pos(i,j)<0.90
%                     prob_50 = [prob_50; [i,j]];
%                 end
%             end
%         end
%     end
% 
%     % plot probability map
%     plot(prob_90(:,1), prob_90(:,2), 'bo'); hold on
%     plot(prob_50(:,1), prob_50(:,2), 'go'); hold on

    % plot sensor positions
    plot(set(:,1), set(:,2), 'ro', 'MarkerFaceColor', 'r'); hold on
    % plot the sensor links
%     for i = 1:size(adj,1)
%         for j = i+1:size(adj,2)
%             if adj(i,j)
%                 plot([set(i,1) set(j,1)], [set(i,2) set(j,2)], 'r');
%             end
%         end
%     end
    % draw communication ranges around them
    for i = 1:size(set,1)
        rectangle('Position',[set(i,1)-radius(i) set(i,2)-radius(i)...
            2*radius(i) 2*radius(i)], 'Curvature', [1 1]);
    end
    % link sensors. They are connected

    % plot obstacles
%     rectangle('Position',[obs_1x obs_1y len_1x len_1y], 'EdgeColor', 'g', 'FaceColor', 'g')
%     rectangle('Position',[obs_2x obs_2y len_2x len_2y], 'EdgeColor', 'g', 'FaceColor', 'g')
%     rectangle('Position',[obs_3x obs_3y len_3x len_3y], 'EdgeColor', 'g', 'FaceColor', 'g')
    % plot(obs_1, 'EdgeColor', 'g', 'FaceColor', 'g'); hold on 
    % plot(obs_2, 'EdgeColor', 'g', 'FaceColor', 'g'); hold on
    % plot(obs_3, 'EdgeColor', 'g', 'FaceColor', 'g'); hold on 
    
    % environment boundary
    axis([b_box(1,1) b_box(1,2) b_box(2,1) b_box(2,2)])
    axis equal
    box on
end