function[IMU,enc] = loadsync(filenameIMU,filenameenc)

clear IMU enc

isenc = 1;
if nargin == 1
    isenc = 0;
    enc = [];
end

dataIMU = dlmread(filenameIMU,'\t',2,0);
nIMU = max(dataIMU(:,1));

flipIMU = 'n';
whichIMU = '';

if nIMU>1
    flipIMU = input('\n Flip IMU? (y/n) ','s');
else
    whichIMU = input('\n Placement IMU? (elb/sho) ','s');
end


if strcmp(flipIMU,'n')
    for ii = 1:nIMU
        IMU(ii).data = dataIMU(dataIMU(:,1)==ii,3:end);
        %IMU(ii).data(:,1:3) = unwrap(IMU(ii).data(:,1:3),180);
        IMU(ii).time = dataIMU(dataIMU(:,1)==ii,2);
        IMU(ii).ts = timeseries(IMU(ii).data,IMU(ii).time);
    end
else
    jj = 1;
    for ii = nIMU:-1:1
        IMU(jj).data = dataIMU(dataIMU(:,1)==ii,3:end);
        %IMU(jj).data(:,1:3) = unwrap(IMU(jj).data(:,1:3),180);
        IMU(jj).time = dataIMU(dataIMU(:,1)==ii,2);
        IMU(jj).ts = timeseries(IMU(jj).data,IMU(jj).time);
        jj = jj+1;
    end
end

h = figure;
for ii = 1:nIMU
    plot(IMU(ii).time,IMU(ii).data(:,3))
    hold on
    xlabel('Time [s]'); ylabel('Angle [deg]');
end

unwrp = input('\n Unwrap IMU angles? (y/n) ','s');
if strcmp(unwrp,'y')
    unwrpan = input('\n Unwrap angle? (-180/180) ');
    for ii = 1:nIMU
        IMU(ii).data(:,1:3) = unwrap(IMU(ii).data(:,1:3),unwrpan);
    end
end
close(h)

ndata = size(IMU(1).data,2);

if isenc
    loadenc = load(filenameenc);
    dataenc = table2array(loadenc.enc);
    enc.time = dataenc(:,1);
    enc.th1 = rad2deg(dataenc(:,2)); % shoulder
    enc.th2 = rad2deg(dataenc(:,3)); % elbow
    
    h = figure;
    if nIMU>1
        subplot(121)
        plot(enc.time,enc.th1)
        hold on
        plot(IMU(1).time,IMU(1).data(:,3))
        xlabel('Time [s]'); ylabel('Angle [deg]')
        legend('Encoder','IMU')
        title('Shoulder')
        subplot(122)
        plot(enc.time,enc.th2)
        hold on
        plot(IMU(2).time,IMU(2).data(:,3))
        xlabel('Time [s]'); ylabel('Angle [deg]')
        legend('Encoder','IMU')
        title('Elbow')
    else
        plot(IMU(1).time,IMU(1).data(:,3))
        hold on
        xlabel('Time [s]'); ylabel('Angle [deg]')
        if strcmp(whichIMU,'elb')
            plot(enc.time,enc.th2)
            title('Elbow')
        elseif strcmp(whichIMU,'sho')
            plot(enc.time,enc.th1)
            title('Shoulder')
        end
        legend('Encoder','IMU')
    end
        
    flip = input('\n Flip encoder? (1/2/b/n) ','s');
    
    close(h)
    
    if strcmp(flip,'n')
        enc.th1c = enc.th1-enc.th1(1);
        enc.th2c = enc.th2-enc.th2(1);
    elseif strcmp(flip,'n')
        enc.th1c = -(enc.th1-enc.th1(1));
        enc.th2c = -(enc.th2-enc.th2(1));
    elseif strcmp(flip,'1')&&strcmp(whichIMU,'sho')
        enc.th1c = -(enc.th1-enc.th1(1));
        enc.th2c = enc.th2-enc.th2(1);
    elseif strcmp(flip,'1')&&strcmp(whichIMU,'elb')
        enc.th1c = enc.th1-enc.th1(1);
        enc.th2c = -(enc.th2-enc.th2(1));
    elseif strcmp(flip,'1')&&nIMU>1
        enc.th1c = -(enc.th1-enc.th1(1));
        enc.th2c = enc.th2-enc.th2(1);
    elseif strcmp(flip,'2')
        enc.th1c = enc.th1-enc.th1(1);
        enc.th2c = -(enc.th2-enc.th2(1));
    end
    enc.ts = timeseries([enc.th1c,enc.th2c],enc.time);
    
    % Synchronizing time vectors
    if nIMU > 1
        [IMU(1).sts,IMU(2).sts] = synchronize(IMU(1).ts,IMU(2).ts,'Intersection');
        [IMU(1).ests,enc.sts] = synchronize(IMU(1).sts,enc.ts,'Intersection');
        [IMU(2).ests,enc.sts] = synchronize(IMU(2).sts,enc.sts,'Intersection');
    else
        [IMU.ests,enc.sts] = synchronize(IMU.ts,enc.ts,'Intersection');
    end
    
    for ii = 1:nIMU
        IMU(ii).stime = IMU(ii).ests.Time;
        IMU(ii).rl = IMU(ii).ests.Data(:,1);
        IMU(ii).pt = IMU(ii).ests.Data(:,2);
        IMU(ii).yw = IMU(ii).ests.Data(:,3);
        IMU(ii).ori = [IMU(ii).yw,IMU(ii).pt,IMU(ii).rl];

        if ndata>3
            IMU(ii).acc = IMU(ii).ests.Data(:,4:6);
            IMU(ii).gyro = rad2deg(IMU(ii).ests.Data(:,7:9));
            IMU(ii).magn = IMU(ii).ests.Data(:,10:12);
            IMU(ii).q.q0 = IMU(ii).ests.Data(:,13);
            IMU(ii).q.q1 = IMU(ii).ests.Data(:,14);
            IMU(ii).q.q2 = IMU(ii).ests.Data(:,15);
            IMU(ii).q.q3 = IMU(ii).ests.Data(:,16);   
            IMU(ii) = EulerfromQuaternion(IMU(ii));
        end
    end
    
    enc.stime = enc.sts.Time;
    enc.scth1 = enc.sts.Data(:,1);
    enc.scth2 = enc.sts.Data(:,2);
    
elseif ~isenc && nIMU == 1
    IMU.stime = IMU.time;
    IMU.rl = IMU.data(:,1);
    IMU.pt = IMU.data(:,2);
    IMU.yw = IMU.data(:,3);
    IMU.ori = [IMU.yw,IMU.pt,IMU.rl];
    
    if ndata>3
        IMU.acc = IMU.ests.Data(:,4:6);
        IMU.gyro = rad2deg(IMU.ests.Data(:,7:9));
        IMU.magn = IMU.ests.Data(:,10:12);
        IMU.q.q0 = IMU.ests.Data(:,13);
        IMU.q.q1 = IMU.ests.Data(:,14);
        IMU.q.q2 = IMU.ests.Data(:,15);
        IMU.q.q3 = IMU.ests.Data(:,16);
        IMU = EulerfromQuaternion(IMU);
    end
    
elseif ~isenc && nIMU > 1
    [IMU(1).sts,IMU(2).sts] = synchronize(IMU(1).ts,IMU(2).ts,'Intersection');
    
    for ii = 1:nIMU
        IMU(ii).stime = IMU(ii).sts.Time;
        IMU(ii).rl = IMU(ii).sts.Data(:,1);
        IMU(ii).pt = IMU(ii).sts.Data(:,2);
        IMU(ii).yw = IMU(ii).sts.Data(:,3);
        IMU(ii).ori = [IMU(ii).yw,IMU(ii).pt,IMU(ii).rl];
        
        if ndata>3
            IMU(ii).acc = IMU(ii).sts.Data(:,4:6);
            IMU(ii).gyro = rad2deg(IMU(ii).sts.Data(:,7:9));
            IMU(ii).magn = IMU(ii).sts.Data(:,10:12);
            IMU(ii).q.q0 = IMU(ii).sts.Data(:,13);
            IMU(ii).q.q1 = IMU(ii).sts.Data(:,14);
            IMU(ii).q.q2 = IMU(ii).sts.Data(:,15);
            IMU(ii).q.q3 = IMU(ii).sts.Data(:,16); 
            IMU(ii) = EulerfromQuaternion(IMU(ii));
        end
    end
end
end
