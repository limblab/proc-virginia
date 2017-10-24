%% Plot of IMU data
addpath('txt');
data = dlmread('20171024_onarm_3D.txt','\t',2,0);

time1 = data(data(:,1)==1,2);
time2 = data(data(:,1)==2,2);
ori1 = data(data(:,1)==1,3:5);
ori2 = data(data(:,1)==2,3:5);
agm1 = data(data(:,1)==1,6:end);
agm2 = data(data(:,1)==2,6:end);

figure
plot(time1,ori1)
hold on
plot(time2,ori2)
legend('Roll_1','Pitch_1','Yaw_1','Roll_2','Pitch_2','Yaw_2')
xlabel('Time [s]'); ylabel('Angle [deg]');

figure
plot(time1,agm1)
legend('ax_1','ay_1','az_1','gx_1','gy_1','gz_1','mx_1','my_1','mz_1')
xlabel('Time [s]'); ylabel('Acc/Gyro/Magn');

%% If IMU started recording before cerebus
timex1 = time1(find(time1<10,1):end,:);
timex2 = time2(find(time2<10,1):end,:);
devx1 = ori1(find(time1<10,1):end,:);
devx2 = ori2(find(time2<10,1):end,:);

figure
plot(timex1,devx1)
hold on
plot(timex2,devx2)
legend('Roll_1','Pitch_1','Yaw_1','Roll_2','Pitch_2','Yaw_2')
xlabel('Time [s]'); ylabel('Angle [deg]');
