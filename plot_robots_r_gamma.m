%% the scripts generate the plot between number robot and radius. 

data_fldr = '/media/ragesh/Disk1/data/resilient_coverage/robots_vs_r/gamma/';

% different gamma values used 
% gammas100 = [51 75 100 120];
gammas100 = [51 120];
% number of trails generated
trails = 10;
col = 'brgk';
gamma100 = gammas100(1);

figure,
hold on

for k =1:length(gammas100)
    i = 1;
    file = [data_fldr num2str(gammas100(k)) '/trail_' num2str(i) '/'];
    radius = csvread([file 'radius_tune_range.csv']);
    rob_data = zeros(trails,length(radius));
    % read data
    for i = 1:trails
        file = [data_fldr num2str(gammas100(k)) '/trail_' num2str(i) '/'];
        data = csvread([file 'robots.csv']);
        rob_data(i,:) = data';
    end
    rob_data_75 = prctile(rob_data,75);
    rob_data_25 = prctile(rob_data,25);
    rob_data_median = prctile(rob_data,50);
    rob_data_mean = mean(rob_data);
    
    ypos = rob_data_75 - rob_data_median;
    yneg = rob_data_median - rob_data_25;
%     plot(radius,rob_data_median);
%     figure
    plot(radius, rob_data_mean, col(k));
    
%      errorbar(radius, rob_data_median,yneg,ypos,'o','MarkerSize',6,...
%     'MarkerEdgeColor',col(k),'MarkerFaceColor',col(k),'LineWidth',2);
end
% legend(['\gamma =',num2str(gammas100(1))],...
%     ['\gamma =',num2str(gammas100(2))],...
%     ['\gamma =',num2str(gammas100(3))],...
%     ['\gamma =',num2str(gammas100(4))],'FontSize',15);
legend(['\gamma =',num2str(gammas100(1)/100)],...
    ['\gamma =',num2str(gammas100(2)/100)],...
    'FontSize',15);
xlabel('Bounding box size L','FontSize',15);
ylabel('Number of robots','FontSize',15);