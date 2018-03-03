%% File selection
lab = -1;
switch lab
    case 0 % mac
        txtpath = '/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt';
        addpath(txtpath);
    case -1 % gob2
        txtpath = [meta.folder,'\IMU\'];
        %txtpath = 'C:\Users\vct1641\Documents\Data\data-IMU\txt';
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

filenames = {[meta.IMUPrefix,meta.taskAlias{1},'.txt'],[meta.IMUPrefix,meta.taskAlias{2},'.txt']};
%filenames = {'20180227_calib_abd_2.txt'};
    
detrnd = [0,0]; % When 1 enables detrend
iscalib = [1,1]; % When 1 only load calibrated data

%% Data loading into IMU struct and plotting angles, accelerations and angular velocities
for  jj = 1:length(filenames)
   
    order = {'back','sho','elb'};  % [back/sho/elb/wrst]
    IMU = loadIMU(filenames{jj},order,detrnd(jj),iscalib(jj));
    filename = strsplit(filenames{jj},'.');
    
    %save(fullfile([meta.folder,'/',filename{1},'.mat']),'IMU');

    opts = {'eul','acc','acc_calib','eul_calib','quat'};
    plotIMU(IMU,filenames{jj},opts);
    
end

%% Get calibration indexes for different poses
clear JA

% Vertical, Flex 90º, Abb 90º
%tpose = [0.1, 0.15, 0.2, 0.25]; % 001 21
%tpose = [0.1, 0.2, 0.32, 0.38]; % 002 21
tpose = [0.05, 0.1, 0.14, 0.18]; % 001 22
%tpose = [0.1, 0.15, 0.25, 0.3]; % 002 22
%tpose = [0.5, 0.55, 0.58, 0.62]; % 001 23
%tpose = [0.1, 0.2, 0.3, 0.4]; % 001 27

%tpose = [400,600,1000,1400];

calibtype = 'FE'; % FE/AA/FE+AA
oritype = 'eul'; % eul/quat
filt = 1; % Use filtered data?
rst = 1; % Reference to OpenSim model?
correct = 0; % Correct for abduction in calibration? - not convincing

JA = getbody2IMUmat(IMU,tpose,calibtype);
JA = getjointangles(IMU,JA,oritype,filt,rst,correct);

%% Plot joint angles and reconstructed global frame IMU angles
opts  = {'joint','body'}; % opts = {'joint','body','joint diff'};
plotJA(IMU,JA,filenames{1},opts);

%% Detrend drift removal
%bkptst = [2 5 16; 8 10 25 ;2 15 25];
%bkptst = [ 1 8 10; 1 4 7 ;1 6  10 ];
bkptst = [3 20 25 ; 5 7 9 ;6 9 20]; % 001 22
%bkptst = [3 8 25 ; 4 7 10 ;6 10 11]; % 002 22
%bkptst = [ 2 10 16 ; 5 7 9; 8 15 25]; % 001 27

plt = 1; % Output plot
IMUfilt = [1,1,1]; % Filter that IMU data?
IMU = dfiltIMU(IMU,bkptst,IMUfilt,plt);

%% Wavelet drift removal parameters
wname = 'haar';
decomplevel = wmaxlev(length(IMU(1).yw),wname);
detaillevel = round(decomplevel/4);

plt = 1;

IMU = wfiltIMU(IMU,wname,detaillevel,plt);
>>>>>>> 0b1aea4f5362f790e143283a536c308a5d012140

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
fprintf('\n Mean ± std of reconstructed global angles IMU %d: \n Roll: %1.3f ± %1.3f\n Pitch: %1.3f ± %1.3f\n Yaw: %1.3f ± %1.3f\n',...
    ii,JA(ii).pks.mrlg,JA(ii).pks.stdrlg,JA(ii).pks.mptg,JA(ii).pks.stdptg,JA(ii).pks.mywg,JA(ii).pks.stdywg);
end


jj = 1;
seg = ['S',num2str(jj)];

while isfield(JA,seg)
    for ii  = 1:size(JA,2)
        fprintf('\n Mean ± std of reconstructed global angles IMU %d, segment %d: \n Roll: %1.3f ± %1.3f\n Pitch: %1.3f ± %1.3f\n Yaw: %1.3f ± %1.3f\n',...
            ii,jj,JA(ii).(seg).pks.mrlg,JA(ii).(seg).pks.stdrlg,JA(ii).(seg).pks.mptg,JA(ii).(seg).pks.stdptg,JA(ii).(seg).pks.mywg,JA(ii).(seg).pks.stdywg);
    end 
    jj = jj+1;
    seg = ['S',num2str(jj)];
end