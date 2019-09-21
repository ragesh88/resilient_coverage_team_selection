%% the scripts generate the plot between number robot and coverage. 

data_fldr = '/media/ragesh/Disk1/data/resilient_coverage/robots_vs_coverage/';

% different gamma values used 
radii = [5 10 15];
% number of trails generated
trails = 5;
robots = 10:10:40;
col = 'brgk';


figure,
hold on

for k =1:length(radii)
%     i = 1;
%     file = [data_fldr num2str(radii(k)) '/trail_' num2str(i) '/'];
    coverage_data = zeros(trails,length(robots));
    % read data
    for i = 1:trails
        file = [data_fldr num2str(radii(k)) '/trail_' num2str(i) '/'];
        data = csvread([file 'coverage.csv']);
        coverage_data(i,:) = data';
    end
    coverage_data_75 = prctile(coverage_data,75);
    coverage_data_25 = prctile(coverage_data,25);
    coverage_data_median = prctile(coverage_data,50);
    coverage_data_mean = mean(coverage_data);
    
    ypos = coverage_data_75 - coverage_data_median;
    yneg = coverage_data_median - coverage_data_25;
%     plot(radius,rob_data_median);
%     figure
%     plot(robots, coverage_data_mean);
     errorbar(robots, coverage_data_median,yneg,ypos,'o','MarkerSize',6,...
    'MarkerEdgeColor',col(k),'MarkerFaceColor',col(k),'LineWidth',2);
end



