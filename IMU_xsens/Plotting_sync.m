%% Load data path
datapath = 'C:\Users\vct1641\Documents\data-IMU\';
addpath(datapath);

%% Handle 
filepath = 'C:\Users\vct1641\Documents\data-IMU\cbmex\';
filename = '20171005_IMU_elbsho';
cds = commonDataStructure(); % Breakpt kinematicsFromNEV, line 85
cds.file2cds([filepath,filename],'arrayIMU','taskRW',6);

%% Data loading
filenameIMU = '20171005_IMU_elbsho.txt';
filenameenc = '20171005_enc_elbsho.mat';

data = dlmread(filenameIMU,'\t',2,0);
timeIMU1 = data(data(:,1)==1,2);
timeIMU2 = data(data(:,1)==2,2);
IMU1 = data(data(:,1)==1,3:end);
IMU2 = data(data(:,1)==2,3:end);

loadenc = load(filenameenc);
enc = table2array(loadenc.enc);
timeenc = enc(:,1);
th1 = rad2deg(enc(:,2));
th2 = rad2deg(enc(:,3));

%% Plotting IMU and encoder
figure
subplot(121)
plot(timeenc,th2-th2(1))
hold on
plot(timeIMU1,IMU1(:,3))
xlabel('Time [s]'); ylabel('Angle [deg]')
legend('Encoder','IMU')
title('Elbow')
subplot(122)
plot(timeenc,-(th1-th1(1)))
hold on
plot(timeIMU2,IMU2(:,3))
xlabel('Time [s]'); ylabel('Angle [deg]')
legend('Encoder','IMU')
title('Shoulder')

%% Time start
iniIMU = timeIMU1(find(diff(IMU1(:,3))>0.01,1));
inienc = timeenc(find(diff(th2)>0.01,1));

%% Plot IMU 
figure
plot(timeIMU1,IMU1)
hold on
plot(timeIMU2,IMU2)
legend('Roll_1','Pitch_1','Yaw_1','Roll_2','Pitch_2','Yaw_2')
xlabel('Time [s]'); ylabel('Angle [deg]');



