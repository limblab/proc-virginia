%% Plot

data = dlmread('20170928_Xsens_cbmex.txt','\t',2,0);

time1 = data(data(:,1)==1,2);
time2 = data(data(:,1)==2,2);
dev1 = data(data(:,1)==1,3:end);
dev2 = data(data(:,1)==2,3:end);

figure
plot(time1,dev1)
hold on
plot(time2,dev2)
legend('Roll','Pitch','Yaw','Roll2','Pitch2','Yaw2')