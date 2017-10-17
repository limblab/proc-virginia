%% Plot of IMU data
addpath('txt');
data = dlmread('20171017_onarm_ref.txt','\t',2,0);

time1 = data(data(:,1)==1,2);
time2 = data(data(:,1)==2,2);
dev1 = data(data(:,1)==1,3:end);
dev2 = data(data(:,1)==2,3:end);

% If IMU started recording before cerebus
timex1 = time1(find(time1<10,1):end,:);
timex2 = time2(find(time2<10,1):end,:);
devx1 = dev1(find(time1<10,1):end,:);
devx2 = dev2(find(time2<10,1):end,:);

figure
plot(timex1,devx1)
hold on
plot(timex2,devx2)
legend('Roll_1','Pitch_1','Yaw_1','Roll_2','Pitch_2','Yaw_2')
xlabel('Time [s]'); ylabel('Angle [deg]');
