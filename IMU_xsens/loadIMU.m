function[IMU] = loadIMU(filenameIMU,isrst)

clear IMU

alldataIMU = importdata(filenameIMU,'\t');
IdsIMU = regexprep(alldataIMU.textdata(2:end,1),'\s','');
header = regexprep(alldataIMU.textdata(1,1:end),'\s','');
dataIMU = alldataIMU.data;

nIMU = max(dataIMU(:,1));

order = [];
order = input('\n Order IMUs? [back/sho/elb/wrst] ','s');
strspl = strsplit(order,'/');

for ii = 1:nIMU
    IMU(ii).place = strspl{ii};
    if any(strcmp(header,'DevIDd'))
        IMU(ii).ID = IdsIMU{dataIMU(:,1)==ii};
    end
    IMU(ii).data = dataIMU(dataIMU(:,1)==ii,3:end);
    IMU(ii).time = dataIMU(dataIMU(:,1)==ii,2);
    IMU(ii).ts = timeseries(IMU(ii).data,IMU(ii).time);
    IMU(ii).sts = IMU(ii).ts;
    IMU(ii).fs = round(length(IMU(ii).time)/(IMU(ii).time(end)-IMU(ii).time(1)));
end

if nIMU > 1
    for ii = 1:nIMU-1
        for jj = ii+1:nIMU
            [IMU(ii).sts,IMU(jj).sts] = synchronize(IMU(ii).sts,IMU(jj).sts,'Intersection');
        end
    end
    for ii = nIMU:-1:2
        [IMU(ii).sts,IMU(1).sts] = synchronize(IMU(ii).sts,IMU(1).sts,'Intersection');
    end
end

it = find(strcmp(header,'CerebusTime'));

flow = 4;
forder = 2;

for ii = 1:nIMU
    IMU(ii).stime = IMU(ii).sts.Time;
    IMU(ii).stimem = (IMU(ii).stime-IMU(ii).stime(1))/60;
    [b,a] = butter(forder,flow*2/IMU(ii).fs,'low');

    if any(strcmp(header,'Roll'))
        irl = find(strcmp(header,'Roll'))-it;
        IMU(ii).rl = IMU(ii).sts.Data(:,irl);
    end
    if any(strcmp(header,'Pitch'))
        ipt = find(strcmp(header,'Pitch'))-it;
        IMU(ii).pt = IMU(ii).sts.Data(:,ipt);
    end
    if any(strcmp(header,'Yaw'))
        iyw = find(strcmp(header,'Yaw'))-it;
        IMU(ii).yw = IMU(ii).sts.Data(:,iyw);
    end
    if any(strcmp(header,'Yaw')) && any(strcmp(header,'Pitch')) && any(strcmp(header,'Roll'))
        IMU(ii).ori = [IMU(ii).rl,IMU(ii).pt,IMU(ii).yw];
        IMU(ii).filt.rl = filtfilt(b,a,IMU(ii).rl);
        IMU(ii).filt.pt = filtfilt(b,a,IMU(ii).pt);
        IMU(ii).filt.yw = filtfilt(b,a,IMU(ii).yw);
        IMU(ii).filt.ori = [IMU(ii).filt.rl,IMU(ii).filt.pt,IMU(ii).filt.yw];
    end
    
    if any(strcmp(header,'xAcc'))
        iac = find(strcmp(header,'xAcc'))-it;
        IMU(ii).acc = IMU(ii).sts.Data(:,iac:iac+2);
    end
    if any(strcmp(header,'xGyro'))
        igy = find(strcmp(header,'xGyro'))-it;
        IMU(ii).gyro = rad2deg(IMU(ii).sts.Data(:,igy:igy+2));
    end
    if any(strcmp(header,'xMagn'))
        img = find(strcmp(header,'xMagn'))-it;
        IMU(ii).magn = IMU(ii).sts.Data(:,img:img+2);
            for j = 1:length(IMU(ii).stime)
                IMU(ii).nmagn(j) = norm(IMU(ii).magn(j,:));
            end
    end
    if any(strcmp(header,'q0'))
        iq = find(strcmp(header,'q0'))-it;
        IMU(ii).q.q0 = IMU(ii).sts.Data(:,iq);
        IMU(ii).q.q1 = IMU(ii).sts.Data(:,iq+1);
        IMU(ii).q.q2 = IMU(ii).sts.Data(:,iq+2);
        IMU(ii).q.q3 = IMU(ii).sts.Data(:,iq+3);
        IMU(ii) = EulfromQuat_IMUload(IMU(ii));
        IMU(ii).filt.q.rl = filtfilt(b,a,IMU(ii).q.rl);
        IMU(ii).filt.q.pt = filtfilt(b,a,IMU(ii).q.pt);
        IMU(ii).filt.q.yw = filtfilt(b,a,IMU(ii).q.yw);
    end
end

if ~isrst
    for ii = 1:nIMU
        IMU(ii).rl = detrend(IMU(ii).rl);
        IMU(ii).pt = detrend(IMU(ii).pt);
        IMU(ii).yw = detrend(IMU(ii).yw);
        IMU(ii).ori = detrend(IMU(ii).ori);
        IMU(ii).q.rl = detrend(IMU(ii).q.rl);
        IMU(ii).q.pt = detrend(IMU(ii).q.pt);
        IMU(ii).q.yw = detrend(IMU(ii).q.yw);
    end
end

end
