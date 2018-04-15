function[IMU] = loadIMU(filenameIMU,order,iscalib,nfile,nCB)
% Loads IMU data into IMU struct from txt file 

% IMU: IMU data structure
% filenameIMU: filename for IMU txt file
% order: order of IMU placement [back/sho/elb/wrst]
% iscalib: whether to only load calibration data when simultaneous
% recording with Cerebus
% nfile: Cerebus file number
% nCB: Number of Cerebi used

clear IMU

% Read file
alldataIMU = importdata(filenameIMU,'\t');
IdsIMU = regexprep(alldataIMU.textdata(2:end,1),'\s','');
header = regexprep(alldataIMU.textdata(1,1:end),'\s','');
dataIMU = alldataIMU.data;

% Number of IMUs
nIMU = max(dataIMU(:,1));

for ii = 1:nIMU
    % IMU ID number
    if any(strcmp(header,'DevIDd'))
        IMU(ii).ID = IdsIMU{dataIMU(:,1)==ii};
    end
    % IMU placement site - back/sho/elb/wrst
    IMU(ii).place = order{ii};
    
    % Separates time and data for each IMU
    time = dataIMU(dataIMU(:,1)==ii,2);
    data = dataIMU(dataIMU(:,1)==ii,3:end);
    
    % Check if Cerebus file was recorded simultaneously, may have 2 time
    % discontinuities (time set to zero): when start recording and when Cerebi sync
    if any(diff(time)<0)
        % Indexes when Cerebus recording time was set to zero
        idx_cbrec = find(diff(time)<0);
        if nCB > 1 % Only consider indexes from sync with 2CB
            idx_sync = find(diff(idx_cbrec)<1000);
            idx_cbrec = sort([idx_cbrec(idx_sync);idx_cbrec(idx_sync)+1]);
        end
        idx_cbrec = [idx_cbrec; length(time)];
        if iscalib % Load only calibration data before Cerebus start recording
            IMU(ii).time = time(1:idx_cbrec(1)-1);
            IMU(ii).data = data(1:idx_cbrec(1)-1,:);
        else
            % Save time, time in min, all data, euler angles, accelerations
            % and quaterions from start of IMU recording to start of
            % Cerebus recording (IMU calibration phase)
            IMU(ii).time_calib = time(1:idx_cbrec(1));
            IMU(ii).timem_calib = (IMU(ii).time_calib-IMU(ii).time_calib(1))/60;
            IMU(ii).data_calib = data(1:idx_cbrec(1),:);
            IMU(ii).eul_calib = IMU(ii).data_calib(:,1:3);
            IMU(ii).acc_calib = IMU(ii).data_calib(:,4:6);
            IMU(ii).q_calib = IMU(ii).data_calib(:,13:16);
            % Save non-calibration data
            if length(idx_cbrec) >= 3 && nCB == 1 % 1CB, 2CB files for 1IMU file
                IMU(ii).time = time(idx_cbrec(nfile)+1:idx_cbrec(nfile+1)-1);
                IMU(ii).data = data(idx_cbrec(nfile)+1:idx_cbrec(nfile+1)-1,:);
            elseif length(idx_cbrec) > 3 && nCB > 1 % 2CB, 2CB files for 1IMU file
                IMU(ii).time = time(idx_cbrec(2*nfile)+1:idx_cbrec(2*nfile+1)-1);
                IMU(ii).data = data(idx_cbrec(2*nfile)+1:idx_cbrec(2*nfile+1)-1,:);
            elseif length(idx_cbrec) == 2  && nCB == 1 % 1CB, 2CB files for 2IMU files
                IMU(ii).time = time(idx_cbrec(1)+1:end);
                IMU(ii).data = data(idx_cbrec(1)+1:end,:);
            else % 2CB, 2CB files for 2IMU files
                IMU(ii).time = time(idx_cbrec(2)+1:end); 
                IMU(ii).data = data(idx_cbrec(2)+1:end,:);
            end
        end
    else
        IMU(ii).time = time;
        IMU(ii).data = data;
    end
    
    % Estimated original IMU sampling/update rate
    IMU(ii).fs = round(length(IMU(ii).time)/(IMU(ii).time(end)-IMU(ii).time(1)));
    
    % IMU packet count - check for missing packets. Packet number starts
    % when start of measurement mode and wraps at 65535
    if any(strcmp(header,'Packcount'))
        IMU(ii).pc = IMU(ii).data(:,end);
    end
    
    % Decimate signals to half the sampling rate (60Hz) and round to 10 ms
    % resolution for speeding up synchronization of IMUs
    IMU(ii).timed = round(decimate(IMU(ii).time,2),2);
    for jj = 1:size(IMU(ii).data,2)
        IMU(ii).datad(:,jj) = round(decimate(IMU(ii).data(:,jj),2),2);
    end
    
    % Creates timeseries for synchronization between IMUs
    IMU(ii).ts = timeseries(IMU(ii).datad,IMU(ii).timed);
    IMU(ii).sts = IMU(ii).ts;
end

% Synchronize IMU data with intersection of time vectors
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

% Save synchronized data
for ii = 1:nIMU
    % Orientation reset
    if any(strcmp(header,'rst'))
        irst = find(strcmp(header,'rst'))-it;
        IMU(ii).rst = IMU(ii).sts.Data(:,irst);
    end
    
    % Packet count number
    if any(strcmp(header,'Packcount'))
        ipc = find(strcmp(header,'Packcount'))-it;    
        IMU(ii).spc = IMU(ii).sts.Data(:,ipc);
    end
    
    % Time vector in seconds
    IMU(ii).stime = IMU(ii).sts.Time;
    % Time vector in minutes
    IMU(ii).stimem = (IMU(ii).stime-IMU(ii).stime(1))/60;
    
    % Roll angle
    if any(strcmp(header,'Roll'))
        irl = find(strcmp(header,'Roll'))-it;
        IMU(ii).rl = IMU(ii).sts.Data(:,irl);
    end
    
    % Pitch angle
    if any(strcmp(header,'Pitch'))
        ipt = find(strcmp(header,'Pitch'))-it;
        IMU(ii).pt = IMU(ii).sts.Data(:,ipt);
    end
    
    % Yaw angle
    if any(strcmp(header,'Yaw'))
        iyw = find(strcmp(header,'Yaw'))-it;
        IMU(ii).yw = IMU(ii).sts.Data(:,iyw);
    end
    
    % Orientation matrix [rl, pt, yw]
    if any(strcmp(header,'Yaw')) && any(strcmp(header,'Pitch')) && any(strcmp(header,'Roll'))
        IMU(ii).ori = [IMU(ii).rl,IMU(ii).pt,IMU(ii).yw];
    end
    
    % Acceleration matrix [ax, ay, az]
    if any(strcmp(header,'xAcc'))
        iac = find(strcmp(header,'xAcc'))-it;
        IMU(ii).acc = IMU(ii).sts.Data(:,iac:iac+2);
    end
    
    % Angular velocity matrix [wx, wy, wz]
    if any(strcmp(header,'xGyro'))
        igy = find(strcmp(header,'xGyro'))-it;
        IMU(ii).gyro = rad2deg(IMU(ii).sts.Data(:,igy:igy+2));
        for j = 1:length(IMU(ii).stime)
            IMU(ii).ngyro(j) = norm(IMU(ii).gyro(j,:));
        end
    end
    
    % Magnetic field matrix [mx, my, mz]
    if any(strcmp(header,'xMagn'))
        img = find(strcmp(header,'xMagn'))-it;
        IMU(ii).magn = IMU(ii).sts.Data(:,img:img+2);
            for j = 1:length(IMU(ii).stime)
                IMU(ii).nmagn(j) = norm(IMU(ii).magn(j,:));
            end
    end
    
    % Quaternions into q stuct
    if any(strcmp(header,'q0'))
        iq = find(strcmp(header,'q0'))-it;
        IMU(ii).q.q0 = IMU(ii).sts.Data(:,iq);
        IMU(ii).q.q1 = IMU(ii).sts.Data(:,iq+1);
        IMU(ii).q.q2 = IMU(ii).sts.Data(:,iq+2);
        IMU(ii).q.q3 = IMU(ii).sts.Data(:,iq+3);
        IMU(ii) = EulfromQuat_IMUload(IMU(ii));
    end
end

% Remove mean value of angle data through detrend if detrnd is one
% if detrnd
%     for ii = 1:nIMU
%         IMU(ii).rl = detrend(IMU(ii).rl,'constant');
%         IMU(ii).pt = detrend(IMU(ii).pt,'constant');
%         IMU(ii).yw = detrend(IMU(ii).yw,'constant');
%         IMU(ii).ori = detrend(IMU(ii).ori,'constant');
%         IMU(ii).q.rl = detrend(IMU(ii).q.rl,'constant');
%         IMU(ii).q.pt = detrend(IMU(ii).q.pt,'constant');
%         IMU(ii).q.yw = detrend(IMU(ii).q.yw,'constant');
%     end
% end
end
