function[IMU,enc] = loadsync(filenameIMU,filenameenc)

isenc = 1;
if nargin == 1
    isenc = 0;
    enc = [];
end

dataIMU = dlmread(filenameIMU,'\t',2,0);
nIMU = max(dataIMU(:,1));

for ii = 1:nIMU
    IMU(ii).data = dataIMU(dataIMU(:,1)==ii,3:end);
    IMU(ii).time = dataIMU(dataIMU(:,1)==ii,2);
    IMU(ii).ts = timeseries(IMU(ii).data,IMU(ii).time);
end

if isenc
    loadenc = load(filenameenc);
    dataenc = table2array(loadenc.enc);
    enc.time = dataenc(:,1);
    enc.th1 = rad2deg(dataenc(:,2)); % shoulder
    enc.th2 = rad2deg(dataenc(:,3)); % elbow
    
    figure(h)
    subplot(121)
    plot(enc.time,enc.th2)
    hold on
    plot(IMU(1).time,IMU(1).yw)
    xlabel('Time [s]'); ylabel('Angle [deg]')
    legend('Encoder','IMU')
    title('Elbow')
    subplot(122)
    plot(enc.time,enc.th1)
    hold on
    plot(IMU(2).time,IMU(2).yw)
    xlabel('Time [s]'); ylabel('Angle [deg]')
    legend('Encoder','IMU')
    title('Shoulder')
    close h
    
    flip = input('\nFlip? (1/2/n)','s');
    if strcmp(flip,'n')
        enc.th1c = enc.th1-abs(enc.th1(1));
        enc.th2c = enc.th2-abs(enc.th2(1));
    elseif strcmp(flip,'1')
        enc.th1c = -(enc.th1-abs(enc.th1(1)));
        enc.th2c = enc.th2-abs(enc.th2(1));
    elseif strcmp(flip,'2')
        enc.th1c = enc.th1-abs(enc.th1(1));
        enc.th2c = -(enc.th2-abs(enc.th2(1)));
    end
    enc.ts = timeseries([enc.th1c,enc.th2c],enc.time);
    
    % Synchronizing time vectors
    if nIMU > 1
        [IMU(1).sts,IMU(2).sts] = synchronize(IMU(1).ts,IMU(2).ts,'Intersection');
        [IMU(1).ests,enc.ests] = synchronize(IMU(1).sts,enc.ts,'Intersection');
        [IMU(2).ests,enc.sts] = synchronize(IMU(2).sts,enc.sts,'Intersection');
    else
        [IMU.ests,enc.sts] = synchronize(IMU.sts,enc.sts,'Intersection');
    end
    
    for ii = 1:nIMU
        IMU(ii).stime = IMU(ii).ests.Time;
        IMU(ii).rl = IMU(ii).ests.Data(:,1);
        IMU(ii).pt = IMU(ii).ests.Data(:,2);
        IMU(ii).yw = IMU(ii).ests.Data(:,3);
    end
    
    enc.stime = enc.sts.Time;
    enc.scth1 = enc.sts.Data(:,1);
    enc.scth2 = enc.sts.Data(:,2);
    
elseif ~isenc && nIMU == 1
        IMU.stime = IMU.time;
        IMU.rl = IMU.data(:,1);
        IMU.pt = IMU.data(:,2);
        IMU.yw = IMU.data(:,3);
 
elseif ~isenc && nIMU > 1
    [IMU(1).sts,IMU(2).sts] = synchronize(IMU(1).ts,IMU(2).ts,'Intersection');
    
    for ii = 1:nIMU
        IMU(ii).stime = IMU(ii).sts.Time;
        IMU(ii).rl = IMU(ii).sts.Data(:,1);
        IMU(ii).pt = IMU(ii).sts.Data(:,2);
        IMU(ii).yw = IMU(ii).sts.Data(:,3);
    end
end
end
