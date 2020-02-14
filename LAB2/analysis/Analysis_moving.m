%Imported data from C2:C60600,E2:H60600,R2:T60600,AD2:AF60600 from imu file
%Imported data from C2:C60600,E2:G60600 from mag file
%Imported data from C2:C1518,H2:I1518 from gps file

%Run only once
imu_data = table2array(carimudata);
mag_data = table2array(carmagdata);
gps_data = table2array(cargpsdata);

imu_time = imu_data(:,1);
imu_orientation = imu_data(:,2:5);
imu_ang_vel = imu_data(:,6:8);
imu_lin_acc = imu_data(:,9:11);

mag_time = mag_data(:,1);
mag_measurements = mag_data(:,2:4);

gps_time = gps_data(:,1);
gps_measurements = gps_data(:,2:3);

sz = 40;

%GPS Plot
figure
plot(gps_measurements(:,1), gps_measurements(:,2));title('GPS Plot');
title('Plot of GPS trajectory'); xlabel('UTM Easting'); ylabel('UTM Northing');


Magnetometer data Plot
figure 
subplot(2,2,1)
plot(mag_measurements(:,1),mag_measurements(:,2));
title('Plot of magnetometer readings(X and Y)'); xlabel('X-axis'); ylabel('Y-axis');

subplot(2,2,2)
plot(mag_time,mag_measurements(:,1));
title('Plot of magnetometer X readings wrt time'); xlabel('time'); ylabel('Magnetometer-X');

subplot(2,2,3)
plot(mag_time,mag_measurements(:,2));title('magnetometer data: y v/s t');
title('Plot of magnetometer Y readings wrt time'); xlabel('time'); ylabel('Magnetometer-Y');

subplot(2,2,4)
plot(mag_time,mag_measurements(:,3));title('magnetometer data: z v/s t');
title('Plot of magnetometer Z readings wrt time'); xlabel('time'); ylabel('Magnetometer-Z');


%Bias subtraction from acceleration
bias = [mean(imu_lin_acc(1:2620,1)) mean(imu_lin_acc(1:2620,2)) mean(imu_lin_acc(1:2620,3))];
imu_lin_acc_unbiased = [imu_lin_acc(:,1)-bias(1) imu_lin_acc(:,2)-bias(2) imu_lin_acc(:,3)-bias(3)];

gps_time_lag = imu_time(1)-gps_time(1);
gps_time_matched = gps_time+gps_time_lag;


imu_time_real = imu_time(10400:52300,:);
imu_orientation_real = imu_orientation(10400:52300,:);
imu_ang_vel_real = imu_ang_vel(10400:52300,:);
imu_lin_acc_real = imu_lin_acc_unbiased(10400:52300,:);

mag_time_real = mag_time(10400:52300,:);
mag_measurements_real = mag_measurements(10400:52300,:);

gps_time_real = gps_time_matched(270:1300,:);
gps_measurements_real = gps_measurements(270:1300,:);




%Hard iron effect:
temp_mag_circle = [mag_measurements(52300:58000,1:2)];
figure 
axis equal;
plot(mag_measurements_real(:,1),mag_measurements_real(:,2));
title('Plot of magnetometer readings (Y vs X)'); xlabel('X-readings'); ylabel('Y-readings');

figure
axis equal;
plot(temp_mag_circle(:,1),temp_mag_circle(:,2));
title('Plot of magnetometer readings for circles around Ruggles circle(Y vs X)'); xlabel('X-readings'); ylabel('Y-readings');

alpha = (max(temp_mag_circle(:,1))+min(temp_mag_circle(:,1)))/2;
beta = (max(temp_mag_circle(:,2))+min(temp_mag_circle(:,2)))/2;
mag_corrected_values_real = [mag_measurements_real(:,1)-alpha, mag_measurements_real(:,2)-beta];
temp_mag_circle_corrected = [temp_mag_circle(:,1)-alpha, temp_mag_circle(:,2)-beta];


figure
axis equal;
plot(temp_mag_circle_corrected(:,1),temp_mag_circle_corrected(:,2));
title('Plot of magnetometer readings for circles after adjustments for Hard Iron effects'); xlabel('X-readings'); ylabel('Y-readings');


%Soft iron effect
r = ((temp_mag_circle_corrected(:,1).^2) + (temp_mag_circle_corrected(:,2).^2)).^0.5;
[m_v,m_I] = min(r);
[r_v,r_I] = max(r);

theta = asind(temp_mag_circle_corrected(r_I,2)/r_v)
R = [cosd(theta) sind(theta); -sind(theta) cosd(theta)];
mag_corrected_values_real = R*mag_corrected_values_real';
v_1 = R*temp_mag_circle_corrected';

scale_factor = m_v/r_v;
mag_corrected_values_real(1,:) = mag_corrected_values_real(1,:).*scale_factor;
v_1(1,:) = v_1(1,:).*scale_factor;
theta = -theta;
R = [cosd(theta) sind(theta); -sind(theta) cosd(theta)];
mag_corrected_values_real = R*mag_corrected_values_real;
v_1 = R*v_1;

figure 
axis equal;
plot(v_1(1,:),v_1(2,:));
title('Plot of magnetometer readings for circles after adjustments for Soft Iron effects'); xlabel('X-readings'); ylabel('Y-readings');

figure
axis equal; axis square;
plot(mag_corrected_values_real(1,:),mag_corrected_values_real(2,:));
title('Plot of magnetometer readings after adjustments for Soft and Hard Iron effects'); xlabel('X-readings'); ylabel('Y-readings');

mag_corrected_values_real = mag_corrected_values_real';


Calculating yaw angles:
yaw_mag = atand(mag_corrected_values_real(:,1)./mag_corrected_values_real(:,2));

imu_time_diff = (imu_time_real-imu_time_real(1))./10^9;
yaw_ang_vel_rad = cumtrapz((imu_time_diff), imu_ang_vel_real(:,3));
yaw_ang_vel_deg = wrapTo180(rad2deg(yaw_ang_vel_rad));

eul = quat2eul([imu_orientation_real(:,4), imu_orientation_real(:,1), imu_orientation_real(:,2), imu_orientation_real(:,3)]);
yaw_eul = wrapTo180(rad2deg(eul(:,1)));

figure
plot(imu_time_real, unwrap(yaw_mag));
hold on
plot(imu_time_real, unwrap(yaw_ang_vel_deg));
plot(imu_time_real, unwrap(yaw_eul));
title('Plot of yaw calculated from magnetometer, gyro, and orientation'); legend('Magnetometer','Gyro','Orientation');xlabel('time'); ylabel('yaw angle (degrees)');
hold off


a = 0.4;
comp_fil_yaw = (yaw_mag.*a)+(yaw_ang_vel_deg.*(1-a));

figure
plot(imu_time_real,unwrap(comp_fil_yaw));
hold on
plot(imu_time_real,unwrap(yaw_eul));
title('Plot of yaw calculated using Complimentary filter and from orientation'); legend({'Complimentary Filter','Orientation'}); xlabel('time'); ylabel('yaw angle (degrees)');
hold off


%Part 3.2:
imu_velocity = cumtrapz(imu_time_diff, imu_lin_acc_real(:,1));

gps_velocity = zeros(size(gps_measurements_real,1));
for c = 1:size(gps_measurements_real,1)-1
     temp = gps_measurements_real(c+1,:)-gps_measurements_real(c,:);
     gps_velocity(c) = ((temp(1)^2 +temp(2)^2)^0.5)/((gps_time(c+1)-gps_time(c))/10^9);
end

figure
plot(gps_time_real,gps_velocity);
hold on
plot(imu_time_real(1:size(imu_velocity)), imu_velocity);
title('Plot of velocities from GPS and from IMU'); legend('GPS Velocity','IMU Velocity');xlabel('time'); ylabel('velocity');
hold off


Part 3.3:
y_dot_dot_calculated = imu_ang_vel_real(:,3).*imu_velocity;

figure
plot(imu_time_real, imu_lin_acc_real(:,2));
hold on
plot(imu_time_real, y_dot_dot_calculated);
title('Plot of acceleration of y_dot_dot_observed and w*X_dot'); legend('w*X_dot','y_dot_dot_observed');xlabel('time'); ylabel('acceleration');
hold off



v_e = imu_velocity.*cos(deg2rad(yaw_eul));
v_n = imu_velocity.*sin(deg2rad(yaw_eul));

x_n = cumtrapz(mag_time_real./10^9,v_n);
x_e = cumtrapz(mag_time_real./10^9,v_e);

figure
plot(gps_time_real,gps_measurements_real(:,1)-gps_measurements_real(1,1));
hold on
plot(imu_time_real,x_e);
title('Plot of Displacements obtained from GPS and IMU (X-axis)'); legend('GPS','IMU');xlabel('time'); ylabel('X-axis/UTM Easting');
hold off

figure
plot(gps_time_real,gps_measurements_real(:,2)-gps_measurements_real(1,2));
hold on
plot(imu_time_real,x_n);
title('Plot of Displacements obtained from GPS and IMU (Y-axis)'); legend('GPS','IMU');xlabel('time'); ylabel('Y-axis/UTM Northing');
hold off

th = -50;
R = [cosd(th) sind(th); -sind(th) cosd(th)];
scale = 0.5;

x_e = -1.*x_e;
% x_n = -1.*x_n;

imu_trajectory = scale.*(R*[x_e x_n]');

figure
plot(imu_trajectory(1,:),imu_trajectory(2,:));
hold on
plot(gps_measurements_real(:,1)-gps_measurements_real(1,1),gps_measurements_real(:,2)-gps_measurements_real(1,2));
title('Plot of trajectories obtained from GPS and IMU'); legend('IMU','GPS');xlabel('X-axis/UTM Easting'); ylabel('Y-axis/UTM Northing');
hold off
