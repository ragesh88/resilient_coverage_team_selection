%% the scripts generate the plot between number robot and radius. 

data_fldr = '/media/ragesh/Disk1/data/resilient_coverage/robots_vs_r/';
coverage = 190;

trails = 10;
i = 1;
file = [data_fldr num2str(coverage) '/trail_' num2str(i) '/'];
radius = csvread([file 'radius_tune_range.csv']);
rob_data = zeros(trails,length(radius));
% read data
for i = 1:trails
    file = [data_fldr num2str(coverage) '/trail_' num2str(i) '/'];
    data = csvread([file 'robots.csv']);
    rob_data(i,:) = data';
end
figure,

% plot(radius(2:end),mean(rob_data(:,2:end)))

% figure,

boxplot(rob_data(:,2:end), radius(2:end))
xlabel('bounding box size L','FontSize',15,'FontWeight','bold');
ylabel('number of robots','FontSize',15,'FontWeight','bold');