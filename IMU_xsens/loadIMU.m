function[IMU] = loadIMU(filenameIMU,order,isrst)

clear IMU

alldataIMU = importdata(filenameIMU,'\t');
IdsIMU = regexprep(alldataIMU.textdata(2:end,1),'\s','');
header = regexprep(alldataIMU.textdata(1,1:end),'\s','');
dataIMU = alldataIMU.data;

nIMU = max(dataIMU(:,1));

for ii = 1:nIMU
    IMU(ii).place = order{ii};
    if any(strcmp(header,'DevIDd'))
        IMU(ii).ID = IdsIMU{dataIMU(:,1)==ii};
    end
    IMU(ii).data = dataIMU(dataIMU(:,1)==ii,3:end);
    IMU(ii).time = dataIMU(dataIMU(:,1)==ii,2);
    IMU(ii).ts = timeseries(IMU(ii).data,IMU(ii).time);
    IMU(ii).sts = IMU(ii).ts;
    IMU(ii).fs = round(length(IMU(ii).time)/(IMU(ii).time(end)-IMU(ii).time(1)));
    if any(strcmp(header,'Packcount'))
        IMU(ii).pc = IMU(ii).data(:,end);
        if diff(IMU(ii).pc)~=1
            disp('\nWarning: there are non ordered packets\n')
            [~,ids] = sort(IMU(ii).pc);
            IMU(ii).data = IMU(ii).data(ids);
            IMU(ii).time = IMU(ii).time(ids);
            IMU(ii).ts = IMU(ii).ts(ids);
            IMU(ii).sts = IMU(ii).sts(ids);
        end
    end
end

if nIMU > 1
    for ii = 1:nIMU-1
        for jj = ii+1:nIMU
            [IMU(ii).sts,IMU(jj).sts] = synchronize(IMU(ii).sts,IMU(jj).sts,'Intersection','KeepOriginalTimes',true);
        end
    end
    for ii = nIMU:-1:2
        [IMU(ii).sts,IMU(1).sts] = synchronize(IMU(ii).sts,IMU(1).sts,'Intersection','KeepOriginalTimes',true);
    end
end

it = find(strcmp(header,'CerebusTime'));

for ii = 1:nIMU
    if any(strcmp(header,'rst'))
        irst = find(strcmp(header,'rst'))-it;
        IMU(ii).rst = IMU(ii).sts.Data(:,irst);
    end
    
    if any(strcmp(header,'Packcount'))
        ipc = find(strcmp(header,'Packcount'))-it;    
        IMU(ii).spc = IMU(ii).sts.Data(:,ipc);
    end

    IMU(ii).stime = IMU(ii).sts.Time;
    IMU(ii).stimem = (IMU(ii).stime-IMU(ii).stime(1))/60;
    
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
    end
    if any(strcmp(header,'xAcc'))
        iac = find(strcmp(header,'xAcc'))-it;
        IMU(ii).acc = IMU(ii).sts.Data(:,iac:iac+2);
    end
    if any(strcmp(header,'xGyro'))
        igy = find(strcmp(header,'xGyro'))-it;
        IMU(ii).gyro = rad2deg(IMU(ii).sts.Data(:,igy:igy+2));
        for j = 1:length(IMU(ii).stime)
            IMU(ii).ngyro(j) = norm(IMU(ii).gyro(j,:));
        end
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
