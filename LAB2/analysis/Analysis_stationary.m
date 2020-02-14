%Imported data from C2:C2082,E2:H2082,R2:T2082,AD2:AF2082 from imu file
%Imported data from A2:A2082,E2:G2082 from mag file

%Run only once
% imu_data = table2array(lab2stationaryimu);
% mag_data = table2array(lab2stationarymag);

imu_time = imu_data(:,1);
imu_orientation = imu_data(:,2:5);
imu_ang_vel = imu_data(:,6:8);
imu_lin_acc = imu_data(:,9:11);

mag_time = mag_data(:,1);
mag_measurements = mag_data(:,2:4);

sz = 40;

figure
scatter(imu_time(:,1), imu_ang_vel(:,1), 'b+');
hold on
scatter(imu_time(:,1), imu_ang_vel(:,2), 'g+');
scatter(imu_time(:,1), imu_ang_vel(:,3), 'r+');
title('Plot of stationary data angular velocity'); legend('x','y','z'); xlabel('time'); ylabel('Angular Velocity');
hold off

figure
scatter(imu_time(:,1), imu_lin_acc(:,1), 'b+');
hold on
scatter(imu_time(:,1), imu_lin_acc(:,2), 'g+');
scatter(imu_time(:,1), imu_lin_acc(:,3), 'r+');
title('Plot of stationary data linear acceleration'); legend('x','y','z'); xlabel('time'); ylabel('Linear Acceleration');
hold off

figure
scatter(mag_time(:,1), mag_measurements(:,1), 'b+');
hold on
scatter(mag_time(:,1), mag_measurements(:,2), 'g+');
scatter(mag_time(:,1), mag_measurements(:,3), 'r+');
title('Plot of stationary data: magnetometer readings'); legend('x','y','z'); xlabel('time'); ylabel('Linear Acceleration');
hold off


figure
subplot(2,2,1);
scatter(imu_time(:,1), imu_ang_vel(:,1), 'b+');
hold on
scatter(imu_time(:,1), imu_ang_vel(:,2), 'g+');
scatter(imu_time(:,1), imu_ang_vel(:,3), 'r+');
title('Plot of stationary data: angular velocity'); legend('x','y','z'); xlabel('time'); ylabel('Angular Velocity');
hold off

imu_ang_vel_mean = mean(imu_ang_vel);
imu_ang_vel_cov = cov(imu_ang_vel);

subplot(2,2,2)
histogram(imu_ang_vel(:,1),75); title('Histogram of stationary angular velocity x points'); xlabel('angular velocity-x'); ylabel('frequency');
hold on
plot([imu_ang_vel_mean(1),imu_ang_vel_mean(1)],[0,100]);
hold off

subplot(2,2,3)
histogram(imu_ang_vel(:,2),75);title('Histogram of stationary angular velocity y points'); xlabel('angular velocity-y'); ylabel('frequency');
hold on
plot([imu_ang_vel_mean(2),imu_ang_vel_mean(2)],[0,100]);
hold off

subplot(2,2,4)
histogram(imu_ang_vel(:,3),75);title('Histogram of stationary angular velocity z points'); xlabel('angular velocity-z'); ylabel('frequency');
hold on
plot([imu_ang_vel_mean(3),imu_ang_vel_mean(3)],[0,100]);
hold off



figure
subplot(2,2,1);
scatter(imu_time(:,1), imu_lin_acc(:,1), 'b+');
hold on
scatter(imu_time(:,1), imu_lin_acc(:,2), 'g+');
scatter(imu_time(:,1), imu_lin_acc(:,3), 'r+');
title('Plot of stationary data: linear acceleration'); legend('x','y','z'); xlabel('time'); ylabel('Linear Acceleration');
hold off

imu_lin_acc_mean = mean(imu_lin_acc);
imu_lin_acc_cov = cov(imu_lin_acc);

subplot(2,2,2)
histogram(imu_lin_acc(:,1),40); title('Histogram of stationary linear acceleration x points'); xlabel('linear acceleration-x'); ylabel('frequency');
hold on
plot([imu_lin_acc_mean(1),imu_lin_acc_mean(1)],[0,200]);
hold off

subplot(2,2,3)
histogram(imu_lin_acc(:,2),40);title('Histogram of stationary linear acceleration y points'); xlabel('linear acceleration-y'); ylabel('frequency');
hold on
plot([imu_lin_acc_mean(2),imu_lin_acc_mean(2)],[0,200]);
hold off

subplot(2,2,4)
histogram(imu_lin_acc(:,3),40);title('Histogram of stationary linear acceleration z points'); xlabel('linear acceleration-z'); ylabel('frequency');
hold on
plot([imu_lin_acc_mean(3),imu_lin_acc_mean(3)],[0,150]);
hold off



figure
subplot(2,2,1);
scatter(mag_time(:,1), mag_measurements(:,1), 'b+');
hold on
scatter(mag_time(:,1), mag_measurements(:,2), 'g+');
scatter(mag_time(:,1), mag_measurements(:,3), 'r+');
title('Plot of stationary data: magnetometer readings'); legend('x','y','z'); xlabel('time'); ylabel('Linear Acceleration');
hold off

mag_measurements_mean = mean(mag_measurements);
mag_measurements_cov = cov(mag_measurements);

subplot(2,2,2)
histogram(mag_measurements(:,1),20); title('Histogram of stationary magnetometer x points'); xlabel('magnetometer-x'); ylabel('frequency');
hold on
plot([mag_measurements_mean(1),mag_measurements_mean(1)],[0,500]);
hold off

subplot(2,2,3)
histogram(mag_measurements(:,2),20);title('Histogram of stationary magnetometer y points'); xlabel('magnetometer-y'); ylabel('frequency');
hold on
plot([mag_measurements_mean(2),mag_measurements_mean(2)],[0,400]);
hold off

subplot(2,2,4)
histogram(mag_measurements(:,3),20);title('Histogram of stationary magnetometer z points'); xlabel('magnetometer-z'); ylabel('frequency');
hold on
plot([mag_measurements_mean(3),mag_measurements_mean(3)],[0,400]);
hold off


