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
filename = '20171019_onrobot';
cds = commonDataStructure(); % Breakpt kinematicsFromNEV, line 85
cds.file2cds([filepath,filename],'arrayIMU','taskRW',6);

%% Handle position from cds.kin
x_h = cds.kin.x;
y_h = cds.kin.y;

%% Data loading
filenameIMU = '20171019_onrobot.txt';
filenameenc = '20171019_onrobot.mat';

[IMU,enc] = loadsync(filenameIMU,filenameenc);
iselb = 0;

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
if iselb
    plot(enc.stime/60,enc.scth2)
    title('Elbow')
else
    plot(enc.stime/60,enc.scth1)
    title('Shoulder')
end
hold on
plot(IMU(1).stime/60,IMU(1).yw)
%xlim([4000 4700]);
xlabel('Time [min]'); ylabel('Angle [deg]')
legend('Encoder','IMU')

%% Plot IMU angles
figure
for ii = 1:size(IMU,2)
    plot(IMU(ii).stime,IMU(ii).ests.Data)
    hold on
end
legend('Roll_1','Pitch_1','Yaw_1','Roll_2','Pitch_2','Yaw_2')
xlabel('Time [s]'); ylabel('Angle [deg]');

%% Quantify drift

bin = find(IMU(1).stime>=60,1);
nbin = floor(length(IMU(1).stime)/bin);
tbin = [bin:bin:bin*nbin];

rmse_elb = zeros(1,nbin);
rmse_sho = zeros(1,nbin);
R_elb = zeros(1,nbin);
R_sho = zeros(1,nbin);

for i = 0:nbin-1
    if size(IMU,2)>1
        rmse_elb(i+1) = rms(IMU(2).yw(1+i*bin:bin+i*bin)-enc.scth2(1+i*bin:bin+i*bin));
        rmse_sho(i+1) = rms(IMU(1).yw(1+i*bin:bin+i*bin)-enc.scth1(1+i*bin:bin+i*bin));
        R_elb(i+1) = sum((IMU(2).yw(1+i*bin:bin+i*bin)-mean(IMU(2).yw(1+i*bin:bin+i*bin))).^2)./sum((enc.scth2(1+i*bin:bin+i*bin)-mean(enc.scth2(1+i*bin:bin+i*bin))).^2);
        R_sho(i+1) = sum((IMU(1).yw(1+i*bin:bin+i*bin)-mean(IMU(1).yw(1+i*bin:bin+i*bin))).^2)./sum((enc.scth1(1+i*bin:bin+i*bin)-mean(enc.scth1(1+i*bin:bin+i*bin))).^2);
    else
        if iselb
            rmse_sho(i+1) = rms(IMU(1).yw(1+i*bin:bin+i*bin)-enc.scth2(1+i*bin:bin+i*bin));
            R_sho(i+1) = sum((IMU(1).yw(1+i*bin:bin+i*bin)-mean(IMU(1).yw(1+i*bin:bin+i*bin))).^2)./sum((enc.scth2(1+i*bin:bin+i*bin)-mean(enc.scth2(1+i*bin:bin+i*bin))).^2);
        else
            rmse_sho(i+1) = rms(IMU(1).yw(1+i*bin:bin+i*bin)-enc.scth1(1+i*bin:bin+i*bin));
            R_sho(i+1) = sum((IMU(1).yw(1+i*bin:bin+i*bin)-mean(IMU(1).yw(1+i*bin:bin+i*bin))).^2)./sum((enc.scth1(1+i*bin:bin+i*bin)-mean(enc.scth1(1+i*bin:bin+i*bin))).^2);
        end
    end
end

figure
plot(enc.stime(tbin)/60,rmse_elb,'b*')
hold on
plot(enc.stime(tbin)/60,rmse_sho,'r*')
%plot((1:nbin+1),R_sho,'r-')size(IMU,2)>1
%plot((1:nbin+1),R_elb,'b-')
xlabel('Time [min]'); ylabel('RMSE [deg]');
legend('Elbow','Shoulder')

%% Position handle with encoder and IMU 
lrelb = 28;
lrsho = 24;
lr = lrelb+lrsho;
XE_sho = [];
Xe_elb = [];

for i = 1:length(enc.stime)
    Xe_sho(:,i) = Rotyaw(enc.scth1(i))*[0;-lrsho;0];
    Xe_elb(:,i) = Xe_sho(:,i) + Rotyaw(enc.scth2(i))*[lrelb;0;0];
end

for i = 1:length(enc.stime)
    XI_sho(:,i) = Rotyaw(IMU(1).yw(i))*[0;-lrsho;0];
    XI_elb(:,i) = XI_sho(:,i) + Rotyaw(IMU(2).yw(i))*[lrelb;0;0];
end

xh_enc = Xe_elb(1,:);
yh_enc = Xe_elb(2,:);

xh_IMU = XI_elb(1,:);
yh_IMU = XI_elb(2,:);
%%
figure
%plot(x_h,y_h)
hold on
plot(xh_enc,yh_enc,'r')
plot(xh_IMU,yh_IMU)
axis equal
xlabel('x_{handle}'); ylabel('y_{handle}');

figure
subplot(121)
plot(enc.stime,xh_enc)
hold on
plot(enc.stime,xh_IMU)
xlabel('Time [s]'); ylabel('x_{handle}');
legend('Encoder','IMU')
subplot(122)
plot(enc.stime,yh_enc)
hold on
plot(enc.stime,yh_IMU)
xlabel('Time [s]'); ylabel('y_{handle}');
legend('Encoder','IMU')

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



