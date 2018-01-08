function[IMU,OS] = loadIMU(filenameIMU,isrst)

clear IMU OS

fid = fopen(filenameIMU,'rt');
header = strsplit(fgets(fid));
fclose(fid);

dataIMU = dlmread(filenameIMU,'\t',2,0);
nIMU = max(dataIMU(:,1));

order = [];
order = input('\n Order IMUs? [back/sho/elb/wrst] ','s');
strspl = strsplit(order,'/');

for ii = 1:nIMU
    IMU(ii).place = strspl{ii};
    IMU(ii).data = dataIMU(dataIMU(:,1)==ii,3:end);
    IMU(ii).time = dataIMU(dataIMU(:,1)==ii,2);
    IMU(ii).ts = timeseries(IMU(ii).data,IMU(ii).time);
    IMU(ii).sts = IMU(ii).ts;
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

for ii = 1:nIMU
    IMU(ii).stime = IMU(ii).sts.Time;
    IMU(ii).stimem = (IMU(ii).stime-IMU(ii).stime(1))/60;
    
    if any(strcmp(header,'Roll'))
        irl = find(strcmp(header,'Roll'))-2;
        IMU(ii).rl = IMU(ii).sts.Data(:,irl);
    end
    if any(strcmp(header,'Pitch'))
        ipt = find(strcmp(header,'Pitch'))-2;
        IMU(ii).pt = IMU(ii).sts.Data(:,ipt);
    end
    if any(strcmp(header,'Yaw'))
        iyw = find(strcmp(header,'Yaw'))-2;
        IMU(ii).yw = IMU(ii).sts.Data(:,iyw);
    end
    if any(strcmp(header,'Yaw')) && any(strcmp(header,'Pitch')) && any(strcmp(header,'Roll'))
        IMU(ii).ori = [IMU(ii).rl,IMU(ii).pt,IMU(ii).yw];
    end
    
    if any(strcmp(header,'xAcc'))
        iac = find(strcmp(header,'xAcc'))-2;
        IMU(ii).acc = IMU(ii).sts.Data(:,iac:iac+2);
    end
    if any(strcmp(header,'xGyro'))
        igy = find(strcmp(header,'xGyro'))-2;
        IMU(ii).gyro = rad2deg(IMU(ii).sts.Data(:,igy:igy+2));
    end
    if any(strcmp(header,'xMagn'))
        img = find(strcmp(header,'xMagn'))-2;
        IMU(ii).magn = IMU(ii).sts.Data(:,img:img+2);
    end
    if any(strcmp(header,'q0'))
        iq = find(strcmp(header,'q0'))-2;
        IMU(ii).q.q0 = IMU(ii).sts.Data(:,iq);
        IMU(ii).q.q1 = IMU(ii).sts.Data(:,iq+1);
        IMU(ii).q.q2 = IMU(ii).sts.Data(:,iq+2);
        IMU(ii).q.q3 = IMU(ii).sts.Data(:,iq+3);
        IMU(ii) = EulerfromQuaternionIMUload(IMU(ii));
    end
end

if ~isrst
    for ii = 1:nIMU
        IMU(ii).ori = detrend(IMU(ii).ori);
        IMU(ii).q.rl = detrend(IMU(ii).q.rl);
        IMU(ii).q.pt = detrend(IMU(ii).q.pt);
        IMU(ii).q.yw = detrend(IMU(ii).q.yw);
    end
end

nback = find(strcmp({IMU.place}, 'back') == 1);
nsho = find(strcmp({IMU.place}, 'sho') == 1);
nelb = find(strcmp({IMU.place}, 'elb') == 1);

%     OS.time = IMU(1).stime;
%     
%     OS.shoulder_flexion = IMU(nsho).pt;
%     OS.shoulder_adduction = IMU(nsho).rl-IMU(nsho).yw;
%     OS.shoulder_rotation = IMU(nsho).yw-IMU(nsho).rl;
%     
%     OS.elbow_flexion = IMU(nelb).pt+IMU(nsho).pt;
%     OS.radial_pronation = IMU(nelb).rl-IMU(nsho).yw+IMU(nsho).rl;
%     
%     OS.header = fieldnames(OS);
%     
%     OS.all = [];
%     
%     for ii = 1:length(OS.header)
%         OS.all = [OS.all OS.(OS.header{ii})];
%     end  
    
OS = [];
end
