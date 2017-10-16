%% Sync check between IMUs and encoders
%% Load data path
datapath = 'C:\Users\vct1641\Documents\data-IMU\';
addpath(datapath);

%% Handle 
filepath = 'C:\Users\vct1641\Documents\data-IMU\cbmex\';
filename = '20171012_onrobot';
cds = commonDataStructure(); % Breakpt kinematicsFromNEV, line 85
cds.file2cds([filepath,filename],'arrayIMU','taskRW',6);

%% Data loading
filenameIMU = '20171012_onrobot.txt';
filenameenc = '20171012_onrobot.mat';

data = dlmread(filenameIMU,'\t',2,0);
timeIMU1 = data(data(:,1)==1,2);
timeIMU2 = data(data(:,1)==2,2);
IMU1 = data(data(:,1)==1,3:end);
IMU2 = data(data(:,1)==2,3:end);

loadenc = load(filenameenc);
enc = table2array(loadenc.enc);
timeenc = enc(:,1);
th1 = rad2deg(enc(:,2)); % shoulder
th2 = rad2deg(enc(:,3)); % elbow
th1c = -(th1-abs(th1(1)));

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
plot(timeenc,th1c)
hold on
plot(timeIMU2,IMU2(:,3))
xlabel('Time [s]'); ylabel('Angle [deg]')
legend('Encoder','IMU')
title('Shoulder')

%% Plot only elbow/shoulder
figure
plot(timeenc/60,th1c)
hold on
plot(timeIMU1/60,IMU1(:,3))
%xlim([4000 4700]);
xlabel('Time [s]'); ylabel('Angle [deg]')
legend('Encoder','IMU')
title('Shoulder')

%% Time start
iniIMU = timeIMU1(find(diff(IMU1(:,3))>0.1,1));
inienc = timeenc(find(diff(th2)>0.1,1));

%% Plot IMU 
figure
plot(timeIMU1,IMU1)
hold on
plot(timeIMU2,IMU2)
legend('Roll_1','Pitch_1','Yaw_1','Roll_2','Pitch_2','Yaw_2')
xlabel('Time [s]'); ylabel('Angle [deg]');

%% Quantify drift
tsIMU1 = timeseries(IMU1(:,3),timeIMU1);
tsth1 = timeseries(th1c,timeenc);
[stsIMU1,ststh1] = synchronize(tsIMU1,tsth1,'Intersection');

%% 
time = stsIMU1.Time;
IMU1s = stsIMU1.Data;
th1s = ststh1.Data;

bin = 60;
nbin = floor(time(end)/bin);
drift = [];
for i = 0:nbin
    drift(i+1) = rms(IMU1s(1+i*bin:bin+i*bin)-th1s(1+i*bin:bin+i*bin));
end

figure
plot((1:nbin+1),drift,'*')
xlabel('Time [min]'); ylabel('RMSE [deg]');

%% FFT
IMU1fft = fft(IMU1s);
encfft = fft(th1s);

figure
plot(abs(IMU1fft))
hold on 
plot(abs(encfft),'r')
xlim([0 500])



