%% File selection
lab = 0;
switch lab
    case 0
        addpath('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt');
    case 3
        addpath('E:\IMU data');
end

filenames = {'20180118_up_movem.txt'};
isrst = [1,1,1]; % When 0 enables detrend

%% Data loading into IMU struct and plotting angles, accelerations and angular velocities
for  jj = 1:length(filenames)
   
    IMU = loadIMU(filenames{jj},isrst(jj));
    %IMU = IMUFE11;
    
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
%     
%     % Plot magnetic field
%     figure('name',filenames{jj})
%     for ii = 1:size(IMU,2)
%         subplot(size(IMU,2),1,ii)
%         plot(IMU(ii).stimem,IMU(ii).magn)
%         xlabel('Time [min]'); ylabel('Magnetic Field [-]');
%         legend('m_x','m_y','m_z')
%         title([IMU(ii).place, ' IMU'])
%     end
%     
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

% Wavelet drift removal parameters
wname = 'haar';
decomplevel = wmaxlev(length(IMU(1).yw),wname);
detaillevel = round(decomplevel/4)-1;

plt = 1;

IMU = filtIMU(IMU,flow,forder,wname,detaillevel,plt);

for ii = 1:size(IMU,2)
    CCmat = [IMU(ii).yw IMU(ii).filt.yw];
    CC = corrcoef(CCmat);
    fprintf('\n Euler R%d = %1.3f',ii,CC(1,2))
    
    CCmat = [IMU(ii).q.pt IMU(ii).filt.q.pt];
    CC = corrcoef(CCmat);
    fprintf('\n Quaternions R%d = %1.3f\n',ii,CC(1,2))
end

% figure('name','Filtered Euler')
% for ii = 1:size(IMU,2)
%     subplot(size(IMU,2),1,ii)
%     plot(IMU(ii).stimem,IMU(ii).filt.ori)
%     xlabel('Time [s]'); ylabel('Angle [deg]');
%     legend('Roll','Pitch','Yaw')
%     title([IMU(ii).place, ' IMU'])
% end
% 
% figure('name','Filtered Quaternions')
% for ii = 1:size(IMU,2)
%     subplot(size(IMU,2),1,ii)
%     plot(IMU(ii).stimem,IMU(ii).filt.q.rl)
%     hold on
%     plot(IMU(ii).stimem,IMU(ii).filt.q.pt)
%     plot(IMU(ii).stimem,IMU(ii).filt.q.yw)
%     xlabel('Time [s]'); ylabel('Angle [deg]');
%     legend('Roll','Pitch','Yaw')
%     title([IMU(ii).place, ' IMU'])
% end

%% Get calibration indexes for different poses
clear JA

tpose = [0.04, 0.05, 0.08, 0.12, 0.24, 0.28]; %% Vertical, Flex 90º, Abb 90º
calibtype = 'FE'; % FE/AA/FE+AA
oritype = 'eul'; % eul/quat
filt = 0;

JA = getbody2IMUmat(IMU,tpose,calibtype);
JA = getjointangles(IMU,JA,oritype,filt);

%% Correct Rsb
for ii = 1:size(IMU,2)
    accv = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
    JA(ii).RAA = vec2Rmat([accv(1) 0 accv(3)],[1 0 0]);
    JA(ii).RFE = vec2Rmat([accv(1:2) 0],[1 0 0]);
    JA(ii).Rsbc = JA(ii).RAA*JA(ii).RFE*JA(ii).Rsb;
end

%% Obtaining calibrated joint angles - using corrected Rsb
joints = {'elb','sho'};
nelb = find(strcmp({IMU.place},'elb'));
nsho = find(strcmp({IMU.place},'sho'));
nback = find(strcmp({IMU.place},'back'));

for ii = 1:length(IMU(1).stime)
    for jj = 1:3
        JA(jj).Rgs = ypr2Rmat(IMU(jj).yw(ii),IMU(jj).pt(ii),IMU(jj).rl(ii));
        %JA(jj).Rgs = quat2Rmat(IMU(jj).q.q0(ii),IMU(jj).q.q1(ii),IMU(jj).q.q2(ii),IMU(jj).q.q3(ii)); % Sensor to global matrix
        [JA(jj).ywg(ii),JA(jj).ptg(ii),JA(jj).rlg(ii)] = Rmat2ypr(JA(jj).Rgs*JA(jj).Rsbc); % Reconstructed global IMU angles
    end
    
    JA(1).Rbb(:,:,ii) = inv(JA(nelb).Rgs*JA(nelb).Rsbc)*(JA(nsho).Rgs*JA(nsho).Rsbc); % Segment to segment matrix
    JA(2).Rbb(:,:,ii) = inv(JA(nback).Rgs*JA(nback).Rsbc)*(JA(nsho).Rgs*JA(nsho).Rsbc); % Segment to segment matrix
    
    for kk = 1:2
        [JA(kk).yw(ii),JA(kk).pt(ii),JA(kk).rl(ii)] = Rmat2ypr(JA(kk).Rbb(:,:,ii)); % Reconstructed joint angles
        JA(kk).place = joints{kk};
    end
    
end

%% Plot joint angles and reconstructed global frame IMU angles

wname = 'haar';
decomplevel = wmaxlev(length(JA(1).rl),wname);
detaillevel = round(decomplevel/4)-1;

for ii = 1:2
[bsline,JA(ii).filt.rl] = wdriftcorrect(JA(ii).rl,wname,detaillevel,decomplevel);
end

figure('name',[filenames{1}, '-Joint Angles'])
subplot(2,1,1)
plot(IMU(1).stimem,(JA(1).filt.rl))
hold on
plot(IMU(1).stimem,(JA(1).pt))
%plot(IMU(1).stimem,unwrap(JA(1).yw))
xlabel('Time [min]'); ylabel('Angle [deg]');
legend('Roll/FE','Pitch/PS/R')
title('elb Joint')
subplot(2,1,2)
plot(IMU(2).stimem,(JA(2).filt.rl))
hold on
plot(IMU(2).stimem,(JA(2).pt))
plot(IMU(2).stimem,(JA(2).yw))
xlabel('Time [min]'); ylabel('Angle [deg]');
legend('Roll/FE','Pitch/PS/R','Yaw/AA')
title('sho Joint')

figure('name',[filenames{1}, '-Reconst Global Angles'])
for ii = 1:size(JA,2)
    subplot(size(JA,2),1,ii)
    plot(IMU(ii).stimem,unwrap(JA(ii).rlg))
    hold on
    plot(IMU(ii).stimem,(JA(ii).ptg))
    plot(IMU(ii).stimem,unwrap(JA(ii).ywg))
    
    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([IMU(ii).place, ' IMU'])
end

%% Find beginning of movement after calibration
forder = 2;
flow = 1;

[b,a] = butter(forder,flow*2/IMU(1).fs,'low');
peaks = [];
for ii = 1:size(IMU,2)
    filtgyro = filtfilt(b,a,IMU(ii).gyro);
    for j = 1:size(IMU(ii).gyro,1)
        IMU(ii).gyrom(j) = norm(filtgyro(j,:));
    end
    thres = max(IMU(ii).gyrom)/2;
    [~, IMU(ii).peaks] = findpeaks((IMU(ii).gyrom), 'minpeakheight', thres);
end

%[~, locs] = findpeaks((gyrom), 'minpeakheight', std(gyrom)/2);

figure
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot(IMU(ii).stimem,IMU(ii).gyro)
    hold on
    plot(IMU(ii).stimem(IMU(ii).peaks(3)), IMU(ii).gyro(IMU(ii).peaks(3),:),'r*')
    xlabel('Time [min]'); ylabel('Angular Velocity [deg/s]');
    legend('w_x','w_y','w_z')
    title([IMU(ii).place, ' IMU'])
end

figure
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot(IMU(ii).stimem,IMU(ii).gyrom)
    xlabel('Time [min]'); ylabel('Angular Velocity [deg/s]');
    title([IMU(ii).place, ' IMU'])
end


