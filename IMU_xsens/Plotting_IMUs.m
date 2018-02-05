%% File selection
lab = 1;
switch lab
    case 0 % mac
        addpath('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt');
    case 1
        addpath('E:\Data-lab1\IMU Data\txt')
    case 3
        addpath('E:\IMU data');
    case 6
        addpath('C:\data\IMU\txt\');
end

filenames = {'20180131_test.txt'};
isrst = [1,1,1]; % When 0 enables detrend

%% Data loading into IMU struct and plotting angles, accelerations and angular velocities
for  jj = 1:length(filenames)
   
    % IMU = loadIMU(filenames{jj},isrst(jj));
    
    % Plot IMU angles from Euler
    figure('name',[filenames{jj}, '-Euler'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).ori)
        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
    
    % Plot IMU angles from quaternions
    figure('name',[filenames{jj}, '-Quaternions'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).q.rl)
        hold on
        plot(IMU(ii).stimem,IMU(ii).q.pt)
        plot(IMU(ii).stimem,IMU(ii).q.yw)
        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
    
    % Plot accelerations
    figure('name',filenames{jj})
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).acc)
        xlabel('Time [min]'); ylabel('Acceleration [m/s^2]');
        legend('a_x','a_y','a_z')
        title([IMU(ii).place, ' IMU'])
    end
    
    % Plot angular velocity
    figure('name',filenames{jj})
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).gyro)
        xlabel('Time [min]'); ylabel('Angular Velocity [deg/s]');
        legend('w_x','w_y','w_z')
        title([IMU(ii).place, ' IMU'])
    end
    
%     % Plot magnetic field
%     figure('name',filenames{jj})
%     for ii = 1:size(IMU,2)
%         subplot(size(IMU,2),1,ii)
%         plot(IMU(ii).stimem,IMU(ii).magn)
%         xlabel('Time [min]'); ylabel('Magnetic Field [-]');
%         legend('m_x','m_y','m_z')
%         title([IMU(ii).place, ' IMU'])
%     end
    
    % Plot normalized magnetic field - should be close to 1
    figure('name',filenames{jj})
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).nmagn)
        xlabel('Time [min]'); ylabel('Normalized Magnetic Field [-]');
        title([IMU(ii).place, ' IMU'])
    end
    
end
%% Filtering IMU data and plotting
% Butter low pass filter parameters
flow = 4;
forder = 2;

%% Wavelet drift removal parameters
wname = 'haar';
decomplevel = wmaxlev(length(IMU(1).yw),wname)-1;
detaillevel = round(decomplevel/4);

plt = 1;

IMU = wfiltIMU(IMU,flow,forder,wname,detaillevel,plt);

%% Detrend drift removal
bkptst = [4;0;0];

for ii = 1:size(IMU,2)
    for j = 1:size(bkptst,2)
        [~,bkpts(ii,j)] = min(abs(IMU(1).stimem-bkptst(ii,j)));
    end
end

plt = 1;

IMU = dfiltIMU(IMU,flow,forder,bkpts,plt);

%% Correlation coefficient 
for ii = 1:size(IMU,2)
    CCmat = [IMU(ii).yw IMU(ii).filt.yw];
    CC = corrcoef(CCmat);
    fprintf('\n Euler R%d = %1.3f',ii,CC(1,2))
    
    CCmat = [IMU(ii).q.pt IMU(ii).filt.q.pt];
    CC = corrcoef(CCmat);
    fprintf('\n Quaternions R%d = %1.3f\n',ii,CC(1,2))
end

%%
figure('name','Filtered Euler')
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot(IMU(ii).stimem,IMU(ii).filt.ori)
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

%% Get calibration indexes for different poses
clear JA

tpose = [0.05, 0.1, 0.15, 0.2, 0.26, 0.34]; %% Vertical, Flex 90�, Abb 90�
%tpose = [0.06, 0.1, 0.15, 0.2, 0.26, 0.34]; %% Vertical, Flex 90�, Abb 90�
calibtype = 'FE'; % FE/AA/FE+AA
oritype = 'quat'; % eul/quat
filt = 0;
rst = 1;

JA = getbody2IMUmat(IMU,tpose,calibtype);
JA = getjointangles(IMU,JA,oritype,filt,rst);

%% Plot joint angles and reconstructed global frame IMU angles

wname = 'haar';
decomplevel = wmaxlev(length(JA(1).rl),wname);
detaillevel = round(decomplevel/4)-1;

for ii = 1:2
    [bsline,JA(ii).filt.rl] = wdriftcorrect(JA(ii).rl,wname,detaillevel,decomplevel);
end

figure('name',[filenames{1}, '-Joint Angles'])
subplot(2,1,1)
plot(IMU(1).stimem,(JA(1).rl))
hold on
%plot(IMU(1).stimem,(JA(1).pt))
plot(IMU(1).stimem,(JA(1).yw))
xlabel('Time [min]'); ylabel('Angle [deg]');
legend('Roll/FE','Pitch/PS/R')
title('elb Joint')
subplot(2,1,2)
plot(IMU(2).stimem,(JA(2).rl))
hold on
plot(IMU(2).stimem,(JA(2).pt))
plot(IMU(2).stimem,unwrap(JA(2).yw))
xlabel('Time [min]'); ylabel('Angle [deg]');
legend('Roll/FE','Pitch/PS/R','Yaw/AA')
title('sho Joint')

figure('name',[filenames{1}, '-Joint Angles diff'])
subplot(2,1,1)
plot(IMU(1).stimem,(JA(1).rld))
hold on
plot(IMU(1).stimem,(JA(1).ptd))
plot(IMU(1).stimem,(JA(1).ywd))
xlabel('Time [min]'); ylabel('Angle [deg]');
legend('Roll/FE','Pitch/PS/R')
title('elb Joint')
subplot(2,1,2)
plot(IMU(2).stimem,(JA(2).rld))
hold on
plot(IMU(2).stimem,(JA(2).ptd))
plot(IMU(2).stimem,unwrap(JA(2).ywd))
xlabel('Time [min]'); ylabel('Angle [deg]');
legend('Roll/FE','Pitch/PS/R','Yaw/AA')
title('sho Joint')

figure('name',[filenames{1}, '-Reconst Global Angles'])
for ii = 1:size(JA,2)
    subplot(size(JA,2),1,ii)
    plot(IMU(ii).stimem,(JA(ii).rlg))
    hold on
    plot(IMU(ii).stimem,(JA(ii).ptg))
    plot(IMU(ii).stimem,(JA(ii).ywg))
    
    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([IMU(ii).place, ' IMU'])
end

%% Validate JA
% (max(JA(ii).rl)-min(JA(ii).rl))/2+min(JA(ii).rl)
% (max(JA(ii).pt)-min(JA(ii).pt))/2+min(JA(ii).pt)
% (max(JA(ii).yw)-min(JA(ii).yw))*2/3+min(JA(ii).yw)

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