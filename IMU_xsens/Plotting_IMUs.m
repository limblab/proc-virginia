%% File selection
lab = 0;
switch lab
    case 0
        addpath('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt');
    case 3
        addpath('E:\IMU data');
end

filenames = {'20180115_calibration_AA.txt'};%,'20171221_calibrationT.txt','20171221_shoelbFE.txt'};
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
    
    % Plot magnetic field
    figure('name',filenames{jj})
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).magn)
        xlabel('Time [min]'); ylabel('Magnetic Field [-]');
        legend('m_x','m_y','m_z')
        title([IMU(ii).place, ' IMU'])
    end
    
    % Plot normalized magnetic field - should be close to 1
    figure('name',filenames{jj})
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).nmagn)
        xlabel('Time [min]'); ylabel('Normalized Magnetic Field [-]');
        title([IMU(ii).place, ' IMU'])
    end
    
end
%% Plotting IMU non detrend
figure
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot(IMU(ii).stime-IMU(ii).stime(1),IMU(ii).sts.Data(:,1:3))
    xlabel('Time [s]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([IMU(ii).place, ' IMU'])
end

%% Calibration with IMU accelerations - sho FE 
JA = [];
%IMU = IMUs21;

for ii = 1:3
    [~,ix1] = min(abs(IMU(ii).stimem-0.04)); % Initial time for 1st pose
    [~,ix2] = min(abs(IMU(ii).stimem-0.06)); % Final time for 1st pose
    [~,ix3] = min(abs(IMU(ii).stimem-0.21)); % Initial time for 2nd pose
    [~,ix4] = min(abs(IMU(ii).stimem-0.25)); % Final time for 2nd pose
    
    zsgA = mean(IMU(ii).acc(ix1:ix2,:));
    zsgB1 = mean(IMU(ii).acc(ix3:ix4,:));
    zsgB = zsgB1/norm(zsgB1);
    zsb = zsgA/norm(zsgA);
    xsb = -cross(zsb,zsgB)/norm(cross(zsb,zsgB));
    ysb = -cross(zsb,xsb)/norm(cross(zsb,xsb));
    
    JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
end

%% Calibration with IMU accelerations - sho AA 
JA = [];
%IMU = IMUs21;

for ii = 1:3
    [~,ix1] = min(abs(IMU(ii).stimem-0.04)); % Initial time for 1st pose
    [~,ix2] = min(abs(IMU(ii).stimem-0.06)); % Final time for 1st pose
    [~,ix3] = min(abs(IMU(ii).stimem-0.11)); % Initial time for 2nd pose
    [~,ix4] = min(abs(IMU(ii).stimem-0.16)); % Final time for 2nd pose
   
    zsgA = mean(IMU(ii).acc(ix1:ix2,:));
    zsgB1 = mean(IMU(ii).acc(ix3:ix4,:));
    zsgB = zsgB1/norm(zsgB1);
    zsb = zsgA/norm(zsgA);
    ysb = -cross(zsb,zsgB)/norm(cross(zsb,zsgB));
    xsb = cross(zsb,ysb)/norm(cross(zsb,ysb));
    
    JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
end

%% Calibration with IMU accelerations - sho FE+AA
JA = [];
%IMU = IMUs21;

for ii = 1:3
    [~,ix1] = min(abs(IMU(ii).stimem-0.04)); % Initial time for 1st pose
    [~,ix2] = min(abs(IMU(ii).stimem-0.06)); % Final time for 1st pose
    [~,ix3] = min(abs(IMU(ii).stimem-0.11)); % Initial time for 2nd pose
    [~,ix4] = min(abs(IMU(ii).stimem-0.16)); % Final time for 2nd pose
    [~,ix5] = min(abs(IMU(ii).stimem-0.2)); % Initial time for 3rd pose
    [~,ix6] = min(abs(IMU(ii).stimem-0.25)); % Final time for 3rd pose
    
    vsgA1 = mean(IMU(ii).acc(ix1:ix2,:));
    vsgB1 = mean(IMU(ii).acc(ix3:ix4,:));
    vsgC1 = mean(IMU(ii).acc(ix5:ix6,:));
    vsgA = vsgA1/norm(vsgA1);
    vsgB = vsgB1/norm(vsgB1);
    vsgC = vsgC1/norm(vsgC1);
    
    ysb = -cross(vsgA,vsgB)/norm(cross(vsgA,vsgB));
    xsb = cross(vsgA,vsgC)/norm(cross(vsgA,vsgC));
    zsb = cross(xsb,ysb)/norm(cross(xsb,ysb));

    JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
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
        [JA(jj).ywg(ii),JA(jj).ptg(ii),JA(jj).rlg(ii)] = Rmat2ypr(JA(jj).Rgs*JA(jj).Rsb); % Reconstructed global IMU angles
    end
    
    JA(1).Rbb(:,:,ii) = inv(JA(nelb).Rgs*JA(nelb).Rsb)*(JA(nsho).Rgs*JA(nsho).Rsb); % Segment to segment matrix
    JA(2).Rbb(:,:,ii) = inv(JA(nback).Rgs*JA(nback).Rsb)*(JA(nsho).Rgs*JA(nsho).Rsb); % Segment to segment matrix
    
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
IMU = IMUFE11(1);
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
%[LO_D,HI_D,LO_R,HI_R] = wfilters('bior1.5');
N = 4;
ord = 14;

[C,L] = wavedec(li,ord,wname);
Ctmp = C;
C(sum(L(1:N)):end) = 0;
Ctmp(1:sum(L(1:N))) = 0;
yA = waverec(C,L,wname);
yD = waverec(Ctmp,L,wname);

figure;
plot(x,li)
hold on
plot(x,yD,'g')
plot(x,yA,'r')
plot(x,yD+median(yA),'m')
