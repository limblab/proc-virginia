
%% Plot of IMU data
data = dlmread('20171011_onarm.txt','\t',2,0);

time1 = data(data(:,1)==1,2);
time2 = data(data(:,1)==2,2);
dev1 = data(data(:,1)==1,3:end);
dev2 = data(data(:,1)==2,3:end);

figure
plot(time1,dev1)
hold on
plot(time2,dev2)
legend('Roll_1','Pitch_1','Yaw_1','Roll_2','Pitch_2','Yaw_2')
xlabel('Time [s]'); ylabel('Angle [deg]');
