%% Sync check between IMUs and encoders
%% Load data path
switch computer
    case 'PCWIN64'
        datapath = 'C:\Users\vct1641\Documents\Data\data-IMU\';
    case 'MACI64'
        datapath = '/Users/virginia/Documents/MATLAB/LIMBLAB/Data';
end

addpath(datapath);

%% Handle 
filepath = 'C:\Users\vct1641\Documents\Data\data-IMU\cbmex\';
filename = '20171017_onrobot';
cds = commonDataStructure(); % Breakpt kinematicsFromNEV, line 85
cds.file2cds([filepath,filename],'arrayIMU','taskCObump',6);

%% Data loading
filenameIMU = '20171017_onrobot.txt';
filenameenc = '20171017_onrobot.mat';

[IMU,enc] = loadsync(filenameIMU,filenameenc);
%%
dataIMU = dlmread(filenameIMU,'\t',2,0);
% timeIMU1 = dataIMU(dataIMU(:,1)==1,2);
% timeIMU2 = dataIMU(dataIMU(:,1)==2,2);
% IMU1 = dataIMU(dataIMU(:,1)==1,3:end);
% IMU2 = dataIMU(dataIMU(:,1)==2,3:end);

for ii = 1:2
    IMU(ii).data = dataIMU(dataIMU(:,1)==ii,3:end);
    IMU(ii).time = dataIMU(dataIMU(:,1)==ii,2);
    IMU(ii).ts = timeseries(IMU(ii).data,IMU(ii).time);
end

% for ii = 1:2
%     IMU(ii).stime = IMU(ii).sts.Time;
%     IMU(ii).rl = IMU(ii).sts.Data(:,1);
%     IMU(ii).pt = IMU(ii).sts.Data(:,2);
%     IMU(ii).yw = IMU(ii).sts.Data(:,3);
% end

% loadenc = load(filenameenc);
% enc = table2array(loadenc.enc);
% timeenc = enc(:,1);
% th1 = rad2deg(enc(:,2)); % shoulder
% th2 = rad2deg(enc(:,3)); % elbow
% th1c = -(th1-abs(th1(1)));
% th2c = th2-abs(th2(1));

loadenc = load(filenameenc);
dataenc = table2array(loadenc.enc);
enc.time = dataenc(:,1);
enc.th1 = rad2deg(dataenc(:,2)); % shoulder
enc.th2 = rad2deg(dataenc(:,3)); % elbow
enc.th1c = -(enc.th1-abs(enc.th1(1)));
enc.th2c = enc.th2-abs(enc.th2(1));
enc.ts = timeseries([enc.th1c,enc.th2c],enc.time);

% Synchronizing time vectors 
[IMU(1).sts,IMU(2).sts] = synchronize(IMU(1).ts,IMU(2).ts,'Intersection');
[IMU(1).ests,enc.ests] = synchronize(IMU(1).sts,enc.ts,'Intersection');
[IMU(2).ests,enc.sts] = synchronize(IMU(2).sts,enc.sts,'Intersection');

for ii = 1:2
    IMU(ii).stime = IMU(ii).ests.Time;
    IMU(ii).rl = IMU(ii).ests.Data(:,1);
    IMU(ii).pt = IMU(ii).ests.Data(:,2);
    IMU(ii).yw = IMU(ii).ests.Data(:,3);
end

enc.stime = enc.sts.Time;
enc.scth1 = enc.sts.Data(:,1);
enc.scth2 = enc.sts.Data(:,2);


%% Plotting IMU and encoder angles
figure
subplot(121)
plot(enc.stime,enc.scth2)
hold on
plot(IMU(1).stime,IMU(1).yw)
xlabel('Time [s]'); ylabel('Angle [deg]')
legend('Encoder','IMU')
title('Elbow')
subplot(122)
plot(enc.stime,enc.scth1)
hold on
plot(IMU(2).stime,IMU(2).yw)
xlabel('Time [s]'); ylabel('Angle [deg]')
legend('Encoder','IMU')
title('Shoulder')

%% Plot only elbow/shoulder angles
figure
plot(enc.stime/60,enc.scth1)
hold on
plot(IMU(1).stime/60,IMU(1).yw)
%xlim([4000 4700]);
xlabel('Time [min]'); ylabel('Angle [deg]')
legend('Encoder','IMU')
title('Shoulder')

%% Plot IMU angles
figure
for ii = 1:2
plot(IMU(ii).stime,IMU(ii).ests.Data)
hold on
end
legend('Roll_1','Pitch_1','Yaw_1','Roll_2','Pitch_2','Yaw_2')
xlabel('Time [s]'); ylabel('Angle [deg]');

%% Quantify drift
tsIMU1 = timeseries(IMU1(:,3),timeIMU1);
tsth1 = timeseries(th1c,timeenc);
[stsIMU1,ststh1] = synchronize(IMU(1).sts,tsth1,'Intersection');

%% 
% time = stsIMU1.Time;
% IMU1s = stsIMU1.Data;
% th1s = ststh1.Data;

bin = 60;
nbin = floor(IMU(1).stime(end)/bin);

drift_elb = zeros(1,nbin+1);
drift_sho = zeros(1,nbin+1);

for i = 0:nbin
    drift_elb(i+1) = rms(IMU(2).yw(1+i*bin:bin+i*bin)-enc.scth2(1+i*bin:bin+i*bin));
    drift_sho(i+1) = rms(IMU(1).yw(1+i*bin:bin+i*bin)-enc.scth1(1+i*bin:bin+i*bin));
end

figure
plot((1:nbin+1),drift_elb,'*')
hold on
plot((1:nbin+1),drift_sho,'*')
xlabel('Time [min]'); ylabel('RMSE [deg]');

%% Position handle
lrelb = 28;
lrsho = 24;
lr = lrelb+lrsho;

for i = 1:length(enc.stime)
    Xr_sho(:,i) = [cosd(enc.scth1(i)) -sind(enc.scth1(i)) 0; sind(enc.scth1(i)) cosd(enc.scth1(i)) 0; 0 0 1]*[0;lrsho;0];
    Xr_elb(:,i) = Xr_sho(:,i) + [cosd(enc.scth2(i)) -sind(enc.scth2(i)) 0; sind(enc.scth2(i)) cosd(enc.scth2(i)) 0; 0 0 1]*[lrelb;0;0];
end

%% Time start
iniIMU = timeIMU1(find(diff(IMU(1).yw)>0.1,1));
inienc = timeenc(find(diff(enc.scth2)>0.1,1));

%% FFT
IMU1fft = fft(IMU1s);
encfft = fft(th1s);

figure
plot(abs(IMU1fft))
hold on 
plot(abs(encfft),'r')
xlim([0 60])



