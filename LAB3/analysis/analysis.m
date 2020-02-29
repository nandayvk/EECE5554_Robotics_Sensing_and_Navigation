%Import the time stamp, altitude, utm_easting, utm_northing, and quality
%from the .csv files

% stationary_isec = table2array(stationaryRTK);
% stationary_field = table2array(stationaryRTKfield);
% walking_isec = table2array(walkingRTK);
% walking_field = table2array(walkingRTKfield);


stationary_isec_mean = [mean(stationary_isec(:,2)),mean(stationary_isec(:,3)),mean(stationary_isec(:,4))];
walking_isec_mean = [mean(walking_isec(:,2)), mean(walking_isec(:,3)), mean(walking_isec(:,4))];
stationary_field_mean = [mean(stationary_field(:,2)),mean(stationary_field(:,3)),mean(stationary_field(:,4))];
walking_field_mean = [mean(walking_field(:,2)),mean(walking_field(:,3)),mean(walking_field(:,4))];


stationary_isec_map_1 = [0,0];
stationary_isec_map_4 = [0,0];
stationary_isec_map_5 = [0,0];
stationary_field_map_1 = [0,0];
stationary_field_map_4 = [0,0];
stationary_field_map_5 = [0,0];
walking_isec_map_1 = [0,0];
walking_isec_map_4 = [0,0];
walking_isec_map_5 = [0,0];
walking_field_map_1 = [0,0];
walking_field_map_4 = [0,0];
walking_field_map_5 = [0,0];

for c = 1:length(stationary_isec)
    if stationary_isec(c,5) == 1;
        stationary_isec_map_1 = [stationary_isec_map_1;stationary_isec(c,3:4)];
    elseif stationary_isec(c,5) == 4;
        stationary_isec_map_4 = [stationary_isec_map_4;stationary_isec(c,3:4)];;
    else stationary_isec(c,5);
        stationary_isec_map_5 = [stationary_isec_map_5;stationary_isec(c,3:4)];;
    end
end

for c = 1:length(stationary_field)
    if stationary_field(c,5) == 1;
        stationary_field_map_1 = [stationary_field_map_1;stationary_field(c,3:4)];
    elseif stationary_field(c,5) == 4;
        stationary_field_map_4 = [stationary_field_map_4;stationary_field(c,3:4)];;
    else stationary_field(c,5);
        stationary_field_map_5 = [stationary_field_map_5;stationary_field(c,3:4)];;
    end
end

for c = 1:length(walking_isec)
    if walking_isec(c,5) == 1;
        walking_isec_map_1 = [walking_isec_map_1;walking_isec(c,3:4)];
    elseif walking_isec(c,5) == 4;
        walking_isec_map_4 = [walking_isec_map_4;walking_isec(c,3:4)];;
    else walking_isec(c,5);
        walking_isec_map_5 = [walking_isec_map_5;walking_isec(c,3:4)];;
    end
end

for c = 1:length(walking_field)
    if walking_field(c,5) == 1;
        walking_field_map_1 = [walking_field_map_1;walking_field(c,3:4)];
    elseif walking_field(c,5) == 4;
        walking_field_map_4 = [walking_field_map_4;walking_field(c,3:4)];;
    else walking_field(c,5);
        walking_field_map_5 = [walking_field_map_5;walking_field(c,3:4)];;
    end
end


sz=40;

%Plots of stationary data points at isec
figure
subplot(2,2,1);
scatter(stationary_isec_map_1(2:10,1), stationary_isec_map_1(2:10,2), sz, 'g+');
hold on
scatter(stationary_isec_map_4(2:6,1), stationary_isec_map_4(2:6,2), sz, 'm+');
scatter(stationary_isec_map_5(2:656,1), stationary_isec_map_5(2:656,2), sz, 'k+');
scatter(stationary_isec_mean(2), stationary_isec_mean(3), sz, 'r+');
title('Plot of stationary data at isec and mean position'); legend('Quality 1','Quality 4','Quality 5','Mean'); xlabel('utm easting'); ylabel('utm northing');
hold off


%Plots of walking data points at isec
subplot(2,2,2);
scatter(walking_isec_map_1(2:49,1), walking_isec_map_1(2:49,2), sz, 'g+');
hold on
% scatter(walking_isec_map_4(2:,1), walking_isec_map_4(2:,2), sz, 'm+');
scatter(walking_isec_map_5(2:48,1), walking_isec_map_5(2:48,2), sz, 'k+');
scatter(walking_isec_mean(2), walking_isec_mean(3), sz, 'r+');
title('Plot of walking data at isec and mean position'); legend('Quality 1','Quality 5','Mean'); xlabel('utm easting'); ylabel('utm northing');
hold off

%Plots of stationary data points at field
subplot(2,2,3);
scatter(stationary_field_map_1(2:10,1), stationary_field_map_1(2:10,2), sz, 'g+');
hold on
scatter(stationary_field_map_4(2:317,1), stationary_field_map_4(2:317,2), sz, 'm+');
scatter(stationary_field_map_5(2:290,1), stationary_field_map_5(2:290,2), sz, 'k+');
scatter(stationary_field_mean(2), stationary_field_mean(3), sz, 'r+');
title('Plot of stationary data at field and mean position'); legend('Quality 1','Quality 4','Quality 5','Mean'); xlabel('utm easting'); ylabel('utm northing');
hold off


%Plots of walking data points at field
subplot(2,2,4);
% scatter(walking_field_map_1(2:,1), walking_field_map_1(2:,2), sz, 'g+');
hold on
% scatter(walking_field_map_4(2:,1), walking_field_map_4(2:,2), sz, 'm+');
scatter(walking_field_map_5(2:175,1), walking_field_map_5(2:175,2), sz, 'k+');
scatter(walking_field_mean(2), walking_field_mean(3), sz, 'r+');
title('Plot of walking data at field and mean position'); legend('Quality 5','Mean'); xlabel('utm easting'); ylabel('utm northing');
hold off


%Histograms of the stationary data
figure
subplot(2,2,1);
histogram(stationary_field(:,2),40); title('Histogram of stationary field data altitude'); xlabel('altitude'); ylabel('frequency');

subplot(2,2,2);
histogram(stationary_field(:,3),40); title('Histogram of stationary field data utm easting'); xlabel('utm easting'); ylabel('frequency');

subplot(2,2,3);
histogram(stationary_field(:,4),40); title('Histogram of stationary field data utm northing'); xlabel('utm northing'); ylabel('frequency');

figure
subplot(2,2,1);
histogram(stationary_isec(:,2),40); title('Histogram of stationary isec data altitude'); xlabel('altitude'); ylabel('frequency');

subplot(2,2,2);
histogram(stationary_isec(:,3),40); title('Histogram of stationary isec data utm easting'); xlabel('utm easting'); ylabel('frequency');

subplot(2,2,3);
histogram(stationary_isec(:,4),40); title('Histogram of stationary isec data utm northing'); xlabel('utm northing'); ylabel('frequency');

norm_stationary_field = stationary_field(:,2:4) - stationary_field_mean;
norm_stationary_isec = stationary_isec(:,2:4) - stationary_isec_mean;

std_dev_stationary_field = std(norm_stationary_field)
std_dev_stationary_isec = std(norm_stationary_isec)

rms_stationary_field = rms(norm_stationary_field)
rms_stationary_isec = rms(norm_stationary_isec)

range_stationary_field = max(stationary_field(:,2:4))-min(stationary_field(:,2:4))
range_stationary_isec = max(stationary_isec(:,2:4))-min(stationary_isec(:,2:4))

% drms_field = 2*sqrt((std_dev_stationary_field(2)^2)+(std_dev_stationary_field(3)^2));
% drms_isec = 2*sqrt((std_dev_stationary_isec(2)^2)+(std_dev_stationary_isec(3)^2));
% 
% figure
% scatter(norm_stationary_field(:,2), norm_stationary_field(:,3));
% hold on
% viscircles([0,0],drms_field);
% hold off
% 
% figure
% scatter(norm_stationary_isec(:,2), norm_stationary_isec(:,3));
% hold on
% viscircles([0,0],drms_isec);
% hold off


figure
subplot(3,2,1);
scatter(stationary_isec(:,1),stationary_isec(:,2));title('time series of stationary isec data altitude'); xlabel('time'); ylabel('altitude');
subplot(3,2,2);
scatter(stationary_isec(:,1),stationary_isec(:,3));title('time series of stationary isec data utm easting'); xlabel('time'); ylabel('utm easting');
subplot(3,2,3);
scatter(stationary_isec(:,1),stationary_isec(:,4));title('time series of stationary isec data utm northing'); xlabel('time'); ylabel('utm northing');
subplot(3,2,4);
scatter(stationary_field(:,1),stationary_field(:,2));title('time series of stationary field data altitude'); xlabel('time'); ylabel('altitude');
subplot(3,2,5);
scatter(stationary_field(:,1),stationary_field(:,3));title('time series of stationary field data utm easting'); xlabel('time'); ylabel('utm easting');
subplot(3,2,6);
scatter(stationary_field(:,1),stationary_field(:,4));title('time series of stationary field data utm northing'); xlabel('time'); ylabel('utm northing');




% p1 = polyfit(([walking_field(1:58,3);walking_field(158:174,3)] - mean([walking_field(1:58,3);walking_field(158:174,3)]))/std([walking_field(1:58,3);walking_field(158:174,3)]),([walking_field(1:58,4);walking_field(158:174,4)] - mean([walking_field(1:58,4);walking_field(158:174,4)]))/std([walking_field(1:58,4);walking_field(158:174,4)]), 1);
% poly1 = polyval(p1,([walking_field(1:58,3);walking_field(158:174,3)]));
% p2 = polyfit((walking_field(58:77,3) - mean(walking_field(58:77,3)))/std(walking_field(58:77,3)),(walking_field(58:77,4) - mean(walking_field(58:77,4)))/std(walking_field(58:77,4)), 1);
% poly2 = polyval(p2,(walking_field(58:77,3)));
% p3 = polyfit((walking_field(78:139,3) - mean(walking_field(78:139,3)))/std(walking_field(78:139,3)),(walking_field(78:139,4) - mean(walking_field(78:139,4)))/std(walking_field(78:139,4)), 1);
% poly3 = polyval(p3,(walking_field(78:139,3)));
% p4 = polyfit((walking_field(140:157,3) - mean(walking_field(140:157,3)))/std(walking_field(140:157,3)),(walking_field(140:157,4) - mean(walking_field(140:157,4)))/std(walking_field(140:157,4)), 1);
% poly4 = polyval(p4,(walking_field(140:157,3)));
% 
% figure
% scatter(walking_field(:,3),walking_field(:,4));
% hold on
% plot(([walking_field(1:58,3);walking_field(158:174,3)]), abs(poly1));
% % hold on
% plot(walking_field(58:77,3), abs(poly2));
% plot(walking_field(78:139,3), abs(poly3));
% plot(walking_field(140:157,3), abs(poly4));
% hold off






% [walking_field(1:58,3);walking_field(158:174,3)],[walking_field(1:58,4);walking_field(158:174,4)];
% (walking_field(58:77,3),walking_field(58:77,4));
% (walking_field(78:139,3),walking_field(78:139,4));
% (walking_field(140:157,3),walking_field(140:157,4));

