%% File selection
lab = 0;
switch lab
    case 0
        addpath('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt');
    case 3
        addpath('E:\IMU data');
end

filenames = {'20180108_stability_g1_1.txt'};
isrst = [1,1,1]; % When 0 enables detrend

%% Data loading into IMU struct and plotting angles, accelerations and angular velocities
for  jj = 1:length(filenames)
   
    %IMU = loadIMU(filenames{jj},isrst(jj));
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
    
%     % Plot angular velocity
%     figure('name',filenames{jj})
%     for ii = 1:size(IMU,2)
%         subplot(size(IMU,2),1,ii)
%         plot(IMU(ii).stimem,IMU(ii).gyro)
%         xlabel('Time [min]'); ylabel('Angular Velocity [deg/s]');
%         legend('w_x','w_y','w_z')
%         title([IMU(ii).place, ' IMU'])
%     end
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
%     % Plot normalized magnetic field - should be close to 1
%     figure('name',filenames{jj})
%     for ii = 1:size(IMU,2)
%         subplot(size(IMU,2),1,ii)
%         plot(IMU(ii).stimem,IMU(ii).nmagn)
%         xlabel('Time [min]'); ylabel('Normalized Magnetic Field [-]');
%         title([IMU(ii).place, ' IMU'])
%     end
    
end
%% Filtering IMU data and plotting
IMU = filtIMU(IMU,1);

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

for ii = 1:size(IMU,2)
    CCmat = [IMU(ii).yw IMU(ii).filt.yw];
    CC = corrcoef(CCmat);
    fprintf('\n Euler R%d = %1.3f',ii,CC(1,2))
    
    CCmat = [IMU(ii).q.pt IMU(ii).filt.q.pt];
    CC = corrcoef(CCmat);
    fprintf('\n Quaternions R%d = %1.3f\n',ii,CC(1,2))
end

%% Get calibration indexes for different poses
JA = [];
tpose = [0.05, 0.1, 0.11, 0.16, 0.15, 0.2]; %% Vertical, Flex 90º, Abb 90º

for i = 1:(length(tpose)/2)
    ixn = ['ix',num2str(i)];
    [~,JA.ixp.(ixn)] = min(abs(IMU(1).stimem-tpose(i)));
end

%% Calibration with IMU accelerations - sho FE

for ii = 1:size(IMU,2)
    zsgA = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
    zsgB1 = mean(IMU(ii).acc(JA(1).ixp.ix5:JA(1).ixp.ix6,:));
    zsgB = zsgB1/norm(zsgB1);
    zsb = zsgA/norm(zsgA);
    xsb = -cross(zsb,zsgB)/norm(cross(zsb,zsgB));
    ysb = -cross(zsb,xsb)/norm(cross(zsb,xsb));
    
    JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
end

%% Calibration with IMU accelerations - sho AA 

for ii = 1:size(IMU,2)
   
    zsgA = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
    zsgB1 = mean(IMU(ii).acc(JA(1).ixp.ix3:JA(1).ixp.ix4,:));
    zsgB = zsgB1/norm(zsgB1);
    zsb = zsgA/norm(zsgA);
    ysb = -cross(zsb,zsgB)/norm(cross(zsb,zsgB));
    xsb = cross(zsb,ysb)/norm(cross(zsb,ysb));
    
    JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
end

%% Calibration with IMU accelerations - sho FE+AA

for ii = 1:size(IMU,2)
    
    vsgA1 = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
    vsgB1 = mean(IMU(ii).acc(JA(1).ixp.ix3:JA(1).ixp.ix4,:));
    vsgC1 = mean(IMU(ii).acc(JA(1).ixp.ix5:JA(1).ixp.ix6,:));
    vsgA = vsgA1/norm(vsgA1);
    vsgB = vsgB1/norm(vsgB1);
    vsgC = vsgC1/norm(vsgC1);
    
    ysb = -cross(vsgA,vsgB)/norm(cross(vsgA,vsgB));
    xsb = cross(vsgA,vsgC)/norm(cross(vsgA,vsgC));
    zsb = cross(xsb,ysb)/norm(cross(xsb,ysb));

    JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
end

%% Correct Rsb
for ii = 1:size(IMU,2)
    accv = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:))
    JA(ii).RAA = vec2Rmat([accv(1) 0 accv(3)],[1 0 0]);
    JA(ii).RFE = vec2Rmat([accv(1:2) 0],[1 0 0]);
    JA(ii).Rsbc = JA(ii).RAA*JA(ii).RFE*JA(ii).Rsb;
end

%% Obtaining calibrated joint angles
joints = {'elb','sho'};
nelb = find(strcmp({IMU.place},'elb'));
nsho = find(strcmp({IMU.place},'sho'));
nback = find(strcmp({IMU.place},'back'));

% for ii = nback
%     IMU(ii).yw = detrend(IMU(ii).yw,'constant');
%     IMU(ii).pt = detrend(IMU(ii).pt);
%     IMU(ii).rl = detrend(IMU(ii).rl);
% end

for ii = 1:length(IMU(1).stime)
    for jj = 1:3
        %JA(jj).Rgs = ypr2Rmat(IMU(jj).yw(ii),IMU(jj).pt(ii),IMU(jj).rl(ii));
        JA(jj).Rgs = quat2Rmat(IMU(jj).q.q0(ii),IMU(jj).q.q1(ii),IMU(jj).q.q2(ii),IMU(jj).q.q3(ii)); % Sensor to global matrix
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
figure('name',[filenames{1}, '-Joint Angles'])
for ii = 1:size(JA,2)-1
    subplot(size(JA,2)-1,1,ii)
    plot(IMU(ii).stimem,JA(ii).rl)
    hold on
    plot(IMU(ii).stimem,(JA(ii).pt))
    plot(IMU(ii).stimem,(JA(ii).yw))
    
    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Roll/FE','Pitch/PS/R','Yaw/AA')
    title([JA(ii).place, ' Joint'])
end

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

%% Polyfit
IMU = IMU(1);
ini = find(IMU.stimem>=1,1);
fin = find(IMU.stimem>=IMU.stimem(end)-2,1);
li = IMU.yw(ini:fin);
x = IMU.stimem(ini:fin);
fit = polyfit(x,li,1);
y = zeros(length(x),1);
N = length(fit);
npts = length(x);

for  i = 1:N
    y = y + fit(N-i+1)*x.^(i-1);
end

bin = 3000;
bin = find(IMUs22(1).stimem>=1,1);
nbin = floor(npts/bin);
xbin = [bin:bin:bin*nbin];
yavg = [];

for i = 0:nbin-1
    yavg(i+1) = mean(li(1+i*bin:bin+i*bin));
end

%bpt = find(IMUs22(2).stimem>=7.4,1);
yd = detrend(li,'linear');

figure;
plot(x,li)
hold on
plot(x,y,'r')
plot(x(xbin),yavg,'g')
plot(x,yd,'b')

%% Filtering HF Noise
fLow = 0.1;
fs = 120;
[b,a] = butter(6,fLow*2/fs,'low');
yf = filtfilt(b,a,li);
yfm = li-yf;
yd = detrend(li);

figure;
plot(x,li)
hold on
plot(x,yfm,'r')
%plot(x,y,'g')

%% Wavelets
wname = 'haar';
N = 4;
ord = 14;

[yA,yD] = wdriftcorrect(li,wname,N,ord);

figure;
plot(x,li)
hold on
plot(x,yD,'g')
plot(x,yA,'r')
%plot(x,yD+median(yA),'m')
