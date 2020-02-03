stationary = table2array(correctedstationarydata);
straightline = table2array(correctedstraightlinedata);

stationary_true_point=[327511.27511 4689891.89891];
straightline_true_points = [327884.27884 4689644.89644; 327672.27672 4689535.89535; 327515.27515 4689458.89458; 327336.27336 4689366.89366];

stationary_mean = [mean(stationary(:,1)),mean(stationary(:,2))];
straightline_mean = [mean(straightline(:,1)), mean(straightline(:,2))];

fprintf('The variation of the GPS data from the true location is around:')
stationary_variation = stationary_mean-stationary_true_point

fprintf('The variation of the GPS data from the true location is approximately around:')
straightline_variation = straightline(419,:) - straightline_true_points(1,:)

sz=100;

%Plots of stationary data points
figure
scatter(stationary(:,1), stationary(:,2), sz, 'b', 'filled');
hold on
scatter(stationary_mean(1,1), stationary_mean(1,2), sz, 'g', 'filled');
scatter(stationary_true_point(1,1),stationary_true_point(1,2), sz, 'r', 'filled');
title('Plot of stationary data with mean and true value'); legend('Data points','Mean', 'True point'); xlabel('utm_easting'); ylabel('utm_northing');
hold off


%Plots of straight-line-walk data points
figure
scatter(straightline(:,1), straightline(:,2), sz, 'b', 'filled');
hold on
scatter(straightline_mean(1,1), straightline_mean(1,2), sz, 'g', 'filled');
scatter(straightline_true_points(:,1), straightline_true_points(:,2), sz, 'r', 'filled');
title('Plot of straight-line-walk data with mean'); legend('Data points', 'Mean', 'True points'); xlabel('utm_easting'); ylabel('utm_northing');
hold off


%Histograms of the data
figure
subplot(1,2,1);
[N1,edges1] = histcounts(stationary(:,1), 'Normalization','count');
edges1 = edges1(2:end) - (edges1(2)-edges1(1))/2;
plot(edges1, N1); title('Histogram of stationary utm_easting points'); xlabel('utm_easting'); ylabel('frequency');

subplot(1,2,2);
[N2,edges2] = histcounts(stationary(:,2), 'Normalization','count');
edges2 = edges2(2:end) - (edges2(2)-edges2(1))/2;
plot(edges2, N2); title('Histogram of stationary utm_northing points'); xlabel('utm_northing'); ylabel('frequency');

