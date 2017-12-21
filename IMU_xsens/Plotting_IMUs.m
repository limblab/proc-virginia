%% Data loading
addpath('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt');
filenames = {'20171221_shoelbFE.txt'};%,'20171221_calibrationT.txt','20171221_shoelbFE.txt'};
isrst = [1,1,1];

%%
for  jj = 1:length(filenames)
    
    [IMU,OS] = loadIMU_toOS(filenames{jj},isrst(jj));
    %IMU = IMU_nometal2;
    
    % Plot IMU angles from Euler
    figure('name',filenames{jj})
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).ori)
        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
    %suptitle('Euler Angles')
    
    %Plot IMU angles from quaternions
    figure('name',filenames{jj})
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).q.rl)
        hold on
        plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).q.pt)
        plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).q.yw)

        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
%     suptitle('Quaternions')

figure('name',filenames{jj})
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).acc)
    xlabel('Time [min]'); ylabel('Acceleration [m/s^2]');
    legend('a_x','a_y','a_z')
    title([IMU(ii).place, ' IMU'])
end

figure('name',filenames{jj})
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).gyro)
    xlabel('Time [min]'); ylabel('Angular Velocity [deg/s]');
    legend('v_x','v_y','v_z')
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

%% Plotting OS

figure
plot(OS.time,OS.all(:,2:end))
legend(OS.header{2:end})

%% Calibration
[val,stix] = min(abs(IMU.stime-IMU.stime(1)-0.1*60));
[val,dyix] = min(abs(IMU.stime-IMU.stime(1)-0.15*60));

v1 = mean(IMU.acc(2:stix,:));
v1n = v1/norm(v1);

v2 = mean(IMU.gyro(dyix:end,:));
v2n = v2/norm(v2);

v3 = cross(v2n,v1n);
v3n = v3/norm(v3);

v4n = cross(v1n,v3n);

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
legend('Roll','Pitch','Yaw'); title('Quat')

%% Calibration?
Rsb = zeros(3);
Rgs = zeros(3);
RbbE = zeros(3);
RbbS = zeros(3);

for ii = 1:3
    [~,ix1] = min(abs(IMU(ii).stime-IMU(ii).stime(1)-0.06*60));
    [~,ix2] = min(abs(IMU(ii).stime-IMU(ii).stime(1)-0.1*60));
    
    zsgA = mean(IMU(ii).acc(2:ix1,:));
    zsgB1 = mean(IMU(ii).acc(ix2:end,:));
    zsgB = zsgB1/norm(mean(IMU(ii).acc(ix2:end,:)));
    zsb = zsgA/norm(zsgA);
    xsb = cross(zsb,zsgB)/norm(cross(zsb,zsgB));
    ysb = cross(zsb,xsb)/norm(cross(zsb,xsb));
    
    Rsb(:,:,ii) = [xsb' ysb' zsb'];
end

%% 
yE = zeros(1,length(IMU(1).stime));
pE = zeros(1,length(IMU(1).stime));
rE = zeros(1,length(IMU(1).stime));
yS = zeros(1,length(IMU(1).stime));
pS = zeros(1,length(IMU(1).stime));
rS = zeros(1,length(IMU(1).stime));
yge = zeros(1,length(IMU(1).stime));
pge = zeros(1,length(IMU(1).stime));
rge = zeros(1,length(IMU(1).stime));
ygs = zeros(1,length(IMU(1).stime));
pgs = zeros(1,length(IMU(1).stime));
rgs = zeros(1,length(IMU(1).stime));

for ii = 1:length(IMU(1).stime)
    for jj = 1:3
        %Rgs(:,:,jj) = yprtoR(IMU(jj).yw(ii),IMU(jj).pt(ii),IMU(jj).rl(ii));
        Rgs(:,:,jj) = qtoR(IMU(jj).q.q0(ii),IMU(jj).q.q1(ii),IMU(jj).q.q2(ii),IMU(jj).q.q3(ii));
    end
    RbbS(:,:,ii) = inv(Rgs(:,:,1)*Rsb(:,:,1))*Rgs(:,:,2)*Rsb(:,:,2);
    RbbE(:,:,ii) = inv(Rgs(:,:,3)*Rsb(:,:,3))*Rgs(:,:,2)*Rsb(:,:,2);
    
    [yge(ii),pge(ii),rge(ii)] = Rtoypr(Rgs(:,:,3));
    [ygs(ii),pgs(ii),rgs(ii)] = Rtoypr(Rgs(:,:,2));

    [yE(ii),pE(ii),rE(ii)] = Rtoypr(RbbE(:,:,ii));
    [yS(ii),pS(ii),rS(ii)] = Rtoypr(RbbS(:,:,ii));
    
end

%% Plot calibration
figure
subplot(2,1,1)
plot(IMU(1).stime,rE)
hold on 
plot(IMU(1).stime,pE)
plot(IMU(1).stime,yE)
legend('Roll','Pitch','Yaw')
title('elb Joint'); xlabel('Time'); ylabel('Angle')
subplot(2,1,2)
plot(IMU(1).stime,rS)
hold on 
plot(IMU(1).stime,pS)
plot(IMU(1).stime,yS)
legend('Roll','Pitch','Yaw')
title('sho Joint'); xlabel('Time'); ylabel('Angle')

figure
subplot(2,1,1)
plot(IMU(1).stime,rge)
hold on 
plot(IMU(1).stime,pge)
plot(IMU(1).stime,yge)
legend('Roll','Pitch','Yaw')
title('elb IMU'); xlabel('Time'); ylabel('Angle')
subplot(2,1,2)
plot(IMU(1).stime,rgs)
hold on 
plot(IMU(1).stime,pgs)
plot(IMU(1).stime,ygs)
legend('Roll','Pitch','Yaw')
title('sho IMU'); xlabel('Time'); ylabel('Angle')

%%

v1v = [0, 0, 0; xsb];
v3v = [0, 0, 0; zsb];
v4v = [0, 0, 0; ysb];

figure
plot3(v1v(:,1),v1v(:,2),v1v(:,3),'r')
grid on
hold on
plot3(v4v(:,1),v4v(:,2),v4v(:,3),'b')
plot3(v3v(:,1),v3v(:,2),v3v(:,3),'g')

