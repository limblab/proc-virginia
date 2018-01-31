%% File selection
lab = 0;
switch lab
    case 0
        addpath('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt');
    case 1
        addpath('E:\Data-lab1\IMU Data\txt')
    case 3
        addpath('E:\IMU data');
end

filenames = {'20180109.txt'};%,'20171221_calibrationT.txt','20171221_shoelbFE.txt'};
isrst = [1,1,1]; % When 0 enables detrend

%% Data loading into IMU struct and plotting angles, accelerations and angular velocities
for  jj = 1:length(filenames)
   
    [IMU,OS] = loadIMU(filenames{jj},isrst(jj));
    %IMU = IMU_nometal2;
    
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

%% Calibration with IMU accelerations
JA = [];
FE = 1;

for ii = 1:3
    [~,ix1] = min(abs(IMU(ii).stimem-0.04)); % Initial time for 1st pose
    [~,ix2] = min(abs(IMU(ii).stimem-0.07)); % Final time for 1st pose
    [~,ix3] = min(abs(IMU(ii).stimem-0.12)); % Initial time for 2nd pose
    [~,ix4] = min(abs(IMU(ii).stimem-0.20)); % Final time for 2nd pose
    if FE
        zsgA = mean(IMU(ii).acc(ix1:ix2,:));
        zsgB1 = mean(IMU(ii).acc(ix3:ix4,:));
        zsgB = zsgB1/norm(zsgB1);
        zsb = zsgA/norm(zsgA);
        xsb = -cross(zsb,zsgB)/norm(cross(zsb,zsgB));
        ysb = -cross(zsb,xsb)/norm(cross(zsb,xsb));
    else
        
    end
    JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
end

%% Obtaining calibrated joint angles
joints = {'sho','elb'};

for ii = 1:length(IMU(1).stime)
    for jj = 1:3
        JA(jj).Rgs = yprtoR(IMU(jj).yw(ii),IMU(jj).pt(ii),IMU(jj).rl(ii)); % Sensor to global matrix with Euler angles
        %JA(jj).Rgs = qtoR(IMU(jj).q.q0(ii),IMU(jj).q.q1(ii),IMU(jj).q.q2(ii),IMU(jj).q.q3(ii)); % Sensor to global matrix with quaternions
        [JA(jj).ywg(ii),JA(jj).ptg(ii),JA(jj).rlg(ii)] = Rtoypr(JA(jj).Rgs*JA(jj).Rsb); % Reconstructed global IMU angles
    end
    
    for kk = 1:2
        JA(kk).Rbb(:,:,ii) = inv(JA(kk).Rgs*JA(kk).Rsb)*(JA(kk+1).Rgs*JA(kk+1).Rsb); % Segment to segment matrix
        [JA(kk).yw(ii),JA(kk).pt(ii),JA(kk).rl(ii)] = Rtoypr(JA(kk).Rbb(:,:,ii)); % Reconstructed joint angles
        JA(kk).place = joints{kk};
    end
    
end

%% Plot joint angles and reconstructed global frame IMU angles
figure
for ii = 1:size(JA,2)-1
    subplot(size(JA,2)-1,1,ii)
    plot(IMU(ii).stimem,-JA(ii).rl)
    hold on
    plot(IMU(ii).stimem,(JA(ii).pt))
    plot(IMU(ii).stimem,(JA(ii).yw))
    
    xlabel('Time [min]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([JA(ii).place, ' Joint'])
end

figure
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

%% Calibration with IMU angular velocity
[val,stix] = min(abs(IMU(1).stimem-0.05));
[val,dyix] = min(abs(IMU(1).stimem-0.274));

v1 = mean(IMU(1).acc(2:stix,:));
v1n = v1/norm(v1);

v2 = (IMU(1).gyro(dyix,:));
v2n = v2/norm(v2);

v3 = cross(v2n,v1n);
v3n = v3/norm(v3);

v4n = cross(v3n,v1n);

v = [v1n;v4n;v3n];

v1v = [0, 0, 0; v1n];
v2v = [0, 0, 0; v2n];
v3v = [0, 0, 0; v3n];
v4v = [0, 0, 0; v4n];

figure
plot3(v1v(:,1),v1v(:,2),v1v(:,3),'r')
grid on
hold on
plot3(v4v(:,1),v4v(:,2),v4v(:,3),'b')
plot3(v3v(:,1),v3v(:,2),v3v(:,3),'g')

%% Plot calibrated data
cdata = [];
cdataq = [];

for ii = 1: length(IMU.stime)
    cdata(ii,:) = v*IMU.data(ii,1:3)';
    cdataq(ii,:) = v*IMU.q.ori(ii,:)';
    
end

figure
subplot(1,2,1)
plot(IMU.stime-IMU.stime(1),cdata)
legend('Roll','Pitch','Yaw'); title('Euler')
subplot(1,2,2)
plot(IMU.stime-IMU.stime(1),cdataq)
legend('Roll','Pitch','Yaw'); title('Quaternion')
