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
filename = '20171018_onrobot';
cds = commonDataStructure(); % Breakpt kinematicsFromNEV, line 85
cds.file2cds([filepath,filename],'arrayIMU','taskRW',6);

%% Handle position from cds.kin
x_h = cds.kin.x;
y_h = cds.kin.y;

%% Data loading
filenameIMU = '20171018_onrobot.txt';
filenameenc = '20171018_onrobot.mat';
[IMU,enc] = loadsync(filenameIMU,filenameenc);

%% Plotting IMU and encoder angles
figure
subplot(121)
plot(enc.stime,enc.scth1)
hold on
plot(IMU(1).stime,IMU(1).yw)
xlabel('Time [s]'); ylabel('Angle [deg]')
legend('Encoder','IMU')
title('Elbow')
subplot(122)
plot(enc.stime,enc.scth2)
hold on
plot(IMU(2).stime,IMU(2).yw)
xlabel('Time [s]'); ylabel('Angle [deg]')
legend('Encoder','IMU')
title('Shoulder')

%% Plot only elbow/shoulder angles
figure
plot(enc.stime/60,enc.scth2)
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

bin = find(IMU(1).stime>=60,1);
nbin = floor(length(IMU(1).stime)/bin);

rmse_elb = zeros(1,nbin+1);
rmse_sho = zeros(1,nbin+1);
R_elb = zeros(1,nbin+1);
R_sho = zeros(1,nbin+1);

for i = 0:nbin-1
    if size(IMU,2)>1
        rmse_elb(i+1) = rms(IMU(2).yw(1+i*bin:bin+i*bin)-enc.scth2(1+i*bin:bin+i*bin));
        rmse_sho(i+1) = rms(IMU(1).yw(1+i*bin:bin+i*bin)-enc.scth1(1+i*bin:bin+i*bin));
        R_elb(i+1) = sum((IMU(2).yw(1+i*bin:bin+i*bin)-mean(IMU(2).yw(1+i*bin:bin+i*bin))).^2)./sum((enc.scth2(1+i*bin:bin+i*bin)-mean(enc.scth2(1+i*bin:bin+i*bin))).^2);
        R_sho(i+1) = sum((IMU(1).yw(1+i*bin:bin+i*bin)-mean(IMU(1).yw(1+i*bin:bin+i*bin))).^2)./sum((enc.scth1(1+i*bin:bin+i*bin)-mean(enc.scth1(1+i*bin:bin+i*bin))).^2);
    else
        rmse_sho(i+1) = rms(IMU(1).yw(1+i*bin:bin+i*bin)-enc.scth2(1+i*bin:bin+i*bin));
        R_sho(i+1) = sum((IMU(1).yw(1+i*bin:bin+i*bin)-mean(IMU(1).yw(1+i*bin:bin+i*bin))).^2)./sum((enc.scth1(1+i*bin:bin+i*bin)-mean(enc.scth1(1+i*bin:bin+i*bin))).^2);
    end
end

figure
plot((1:nbin+1),rmse_elb,'b*')
hold on
plot((1:nbin+1),rmse_sho,'r*')
%plot((1:nbin+1),R_sho,'r-')
%plot((1:nbin+1),R_elb,'b-')
xlabel('Time [min]'); ylabel('RMSE [deg]');

%% Position handle
lrelb = 28;
lrsho = 24;
lr = lrelb+lrsho;

for i = 1:length(enc.stime)
    Xr_sho(:,i) = Roty(enc.scth1(i))*[lrsho;0;0];
    Xr_elb(:,i) = Xr_sho(:,i) + Roty(-enc.scth2(i))*[0;-lrelb;0];
end

x_rh = Xr_elb(1,:);
y_rh = Xr_elb(2,:);

figure
plot(x_h,y_h)
hold on
plot(x_rh,y_rh,'r')
axis equal

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



