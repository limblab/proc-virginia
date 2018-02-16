%% File selection
lab = 1;
switch lab
    case 0 % mac
        txtpath = '/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt';
        addpath(txtpath);
    case -1
        txtpath = 'C:\Users\vct1641\Documents\Data\data-IMU';
        addpath(txtpath);
    case 1
        txtpath = 'E:\Data-lab1\IMU Data\txt';
        addpath(txtpath);
    case 3
        txtpath = 'E:\IMU data';
        addpath(txtpath);
    case 6
        txtpath = 'C:\data\IMU\txt\';
        addpath(txtpath);
end

filenames = {'20180213_reset30_mag3.txt'};

isrst = [1,1,1]; % When 0 enables detrend

%% Data loading into IMU struct and plotting angles, accelerations and angular velocities
for  jj = 1:length(filenames)
   
    order = {'sho','elb','magn'};  % [back/sho/elb/wrst]
    IMU = loadIMU(filenames{jj},order,isrst(jj));
    
    opts = {'eul','quat','magn'};
    plotIMU(IMU,filenames{jj},opts);
    
end

%% Get calibration indexes for different poses
clear JA

tpose = [0.04, 0.08, 0.12, 0.16, 0.26, 0.34]; %% Vertical, Flex 90�, Abb 90�
calibtype = 'FE'; % FE/AA/FE+AA
oritype = 'eul'; % eul/quat
filt = 0;
rst = 1;
correct = 0;

JA = getbody2IMUmat(IMU,tpose,calibtype);
JA = getjointangles(IMU,JA,oritype,filt,rst,correct);

%% Plot joint angles and reconstructed global frame IMU angles

for jj = 1:length(filenames)
    opts  = {'joint','body'};
    plotJA(IMU,JA,filenames{jj},opts);
end

%% Binding reset segments
IMU = bindrst(IMU);

figure('name',[filenames{1}, '-Binding Euler'])
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot(IMU(ii).stimem,IMU(ii).rstb.yw)    
    hold on
    %plot(IMU(ii).stimem,IMU(ii).rstb.pt)
    %plot(IMU(ii).stimem,IMU(ii).rstb.rl)
    
    plot(IMU(ii).stimem,IMU(ii).yw)
    %plot(IMU(ii).stimem,IMU(ii).pt)
    %plot(IMU(ii).stimem,IMU(ii).rl)

    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Bound yaw','Yaw')
    title([IMU(ii).place, ' IMU'])
end
%%
figure('name',[filenames{1}, '-Binding Quat'])
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot(IMU(ii).stimem,IMU(ii).rstb.q.rl)
    hold on
    plot(IMU(ii).stimem,IMU(ii).rstb.q.pt)
    plot(IMU(ii).stimem,IMU(ii).rstb.q.yw)
    plot(IMU(ii).stimem,IMU(ii).q.rl)
    plot(IMU(ii).stimem,IMU(ii).q.pt)
    plot(IMU(ii).stimem,IMU(ii).q.yw)
    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Roll B','Pitch B','Yaw B','Roll','Pitch','Yaw')
    title([IMU(ii).place, ' IMU'])
end

%% Adding missing fields to previous version of IMU data collection
pla = {'back','sho','elb'};
for ii = 1:size(IMU,2)
IMU(ii).fs = 120;
IMU(ii).stimem = (IMU(ii).stime-IMU(ii).stime(1))/60;
IMU(ii).place = pla{ii};
end

%% Filtering IMU data and plotting
%% Wavelet drift removal parameters
wname = 'haar';
decomplevel = wmaxlev(length(IMU(1).yw),wname);
detaillevel = round(decomplevel/4);

plt = 1;

IMU = wfiltIMU(IMU,wname,detaillevel,plt);

%% Detrend drift removal
bkptst = [0;0;0];

plt = 1;

IMU = dfiltIMU(IMU,bkptst,plt);

%% Correlation coefficient 
for ii = 1:size(IMU,2)
    CCmat = [IMU(ii).yw IMU(ii).filt.yw];
    CC = corrcoef(CCmat);
    fprintf('\n Euler R%d = %1.3f',ii,CC(1,2))
    
    if isfield(IMU,'q')
    CCmat = [IMU(ii).q.pt IMU(ii).filt.q.pt];
    CC = corrcoef(CCmat);
    fprintf('\n Quaternions R%d = %1.3f\n',ii,CC(1,2))
    end
end

%% Plot filtered angles
figure('name','Filtered Euler')
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot(IMU(ii).stimem,IMU(ii).filt.rl)
    hold on
    plot(IMU(ii).stimem,IMU(ii).filt.pt)
    plot(IMU(ii).stimem,IMU(ii).filt.yw)
    xlabel('Time [s]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([IMU(ii).place, ' IMU'])
end


figure('name','Filtered Quaternions')
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot(IMU(ii).stimem,IMU(ii).filt.q.rl)
    hold on
    plot(IMU(ii).stimem,IMU(ii).filt.q.pt)
    plot(IMU(ii).stimem,IMU(ii).filt.q.yw)
    xlabel('Time [s]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([IMU(ii).place, ' IMU'])
end

%% Validate JA
for ii = 1:size(JA,2)-1
    [~, JA(ii).rlpks] = findpeaks((JA(ii).rl), 'minpeakheight', mean(JA(ii).rl)+std(JA(ii).rl)/2,'minpeakdistance',100);
    [~, JA(ii).ptpks] = findpeaks((JA(ii).pt), 'minpeakheight', mean(JA(ii).pt)+std(JA(ii).pt)/2,'minpeakdistance',100);
    [~, JA(ii).ywpks] = findpeaks((JA(ii).yw), 'minpeakheight', mean(JA(ii).yw)+std(JA(ii).yw)/2,'minpeakdistance',100);
end


figure('name',[filenames{1}, '-Joint Angles'])
subplot(2,1,1)
plot(IMU(1).stimem,(JA(1).rl))
hold on
plot(IMU(1).stimem,(JA(1).pt))
plot(IMU(1).stimem,(JA(1).yw))
plot(IMU(1).stimem(JA(1).rlpks), JA(1).rl(JA(1).rlpks),'r*')
plot(IMU(1).stimem(JA(1).ptpks), JA(1).pt(JA(1).ptpks),'r*')
plot(IMU(1).stimem(JA(1).ywpks), JA(1).yw(JA(1).ywpks),'r*')
xlabel('Time [min]'); ylabel('Angle [deg]');
legend('Roll/FE','Pitch/PS/R','Yaw')
title('elb Joint')

subplot(2,1,2)
plot(IMU(2).stimem,(JA(2).rl))
hold on
plot(IMU(2 ).stimem,(JA(2).pt))
plot(IMU(2).stimem,unwrap(JA(2).yw))
plot(IMU(2).stimem(JA(2).rlpks), JA(2).rl(JA(2).rlpks),'r*')
plot(IMU(2).stimem(JA(2).ptpks), JA(2).pt(JA(2).ptpks),'r*')
plot(IMU(2).stimem(JA(2).ywpks), (JA(2).yw(JA(2).ywpks)),'r*')
xlabel('Time [min]'); ylabel('Angle [deg]');
legend('Roll/FE','Pitch/PS/R','Yaw/AA')
title('sho Joint')

%% Get segments for different movements
tJA = [0.3,0.6,0.65,0.9];
JA = getJAsegm(IMU,JA,tJA); 
JA = getpks(JA);

%% Plot reconstructed JA for segments
figure('name',[filenames{1}, '-Reconst GA FE'])
for ii = 1:size(JA,2)
    subplot(size(JA,2),1,ii)
    plot(JA(ii).S1.time,(JA(ii).S1.rlg))
    hold on
    plot(JA(ii).S1.time,(JA(ii).S1.ptg))
    plot(JA(ii).S1.time,(JA(ii).S1.ywg))
    
    plot(JA(ii).S1.pks.trlg,(JA(ii).S1.pks.rlg),'r*')
    plot(JA(ii).S1.pks.tptg,(JA(ii).S1.pks.ptg),'r*')
    plot(JA(ii).S1.pks.tywg,(JA(ii).S1.pks.ywg),'r*')
    
    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([IMU(ii).place, ' IMU - FE seg'])
end

figure('name',[filenames{1}, '-Reconst GA PS'])
for ii = 1:size(JA,2)
    subplot(size(JA,2),1,ii)
    plot(JA(ii).S2.time,(JA(ii).S2.rlg))
    hold on
    plot(JA(ii).S2.time,(JA(ii).S2.ptg))
    plot(JA(ii).S2.time,(JA(ii).S2.ywg))
    
    plot(JA(ii).S2.pks.trlg,(JA(ii).S2.pks.rlg),'r*')
    plot(JA(ii).S2.pks.tptg,(JA(ii).S2.pks.ptg),'r*')
    plot(JA(ii).S2.pks.tywg,(JA(ii).S2.pks.ywg),'r*')
    
    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([IMU(ii).place, ' IMU - PS seg'])
end

%% Plot reconstructed JA for segments
figure('name',[filenames{1}, '-Reconst GA FE'])
for ii = 1:size(JA,2)-1
    subplot(size(JA,2)-1,1,ii)
    plot(JA(ii).S1.time,(JA(ii).S1.rl))
    hold on
    plot(JA(ii).S1.time,(JA(ii).S1.pt))
    plot(JA(ii).S1.time,(JA(ii).S1.yw))
 
    plot(JA(ii).S1.pks.trl,(JA(ii).S1.pks.rl),'r*')
    plot(JA(ii).S1.pks.tpt,(JA(ii).S1.pks.pt),'r*')
    plot(JA(ii).S1.pks.tyw,(JA(ii).S1.pks.yw),'r*')
    
    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([JA(ii).joint, ' - FE seg'])
end

figure('name',[filenames{1}, '-Reconst GA PS'])
for ii = 1:size(JA,2)-1
    subplot(size(JA,2)-1,1,ii)
    plot(JA(ii).S2.time,(JA(ii).S2.rl))
    hold on
    plot(JA(ii).S2.time,(JA(ii).S2.pt))
    plot(JA(ii).S2.time,(JA(ii).S2.yw))
    
    plot(JA(ii).S2.pks.trl,(JA(ii).S2.pks.rl),'r*')
    plot(JA(ii).S2.pks.tpt,(JA(ii).S2.pks.pt),'r*')
    plot(JA(ii).S2.pks.tyw,(JA(ii).S2.pks.yw),'r*')
    
    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([JA(ii).joint, ' - PS seg'])
end

%% Mean and std of peaks
for ii  = 1:size(JA,2)
fprintf('\n Mean � std of reconstructed global angles IMU %d: \n Roll: %1.3f � %1.3f\n Pitch: %1.3f � %1.3f\n Yaw: %1.3f � %1.3f\n',...
    ii,JA(ii).pks.mrlg,JA(ii).pks.stdrlg,JA(ii).pks.mptg,JA(ii).pks.stdptg,JA(ii).pks.mywg,JA(ii).pks.stdywg);
end


jj = 1;
seg = ['S',num2str(jj)];

while isfield(JA,seg)
    for ii  = 1:size(JA,2)
        fprintf('\n Mean � std of reconstructed global angles IMU %d, segment %d: \n Roll: %1.3f � %1.3f\n Pitch: %1.3f � %1.3f\n Yaw: %1.3f � %1.3f\n',...
            ii,jj,JA(ii).(seg).pks.mrlg,JA(ii).(seg).pks.stdrlg,JA(ii).(seg).pks.mptg,JA(ii).(seg).pks.stdptg,JA(ii).(seg).pks.mywg,JA(ii).(seg).pks.stdywg);
    end 
    jj = jj+1;
    seg = ['S',num2str(jj)];
end