function[IMU,OS] = loadIMU_toOS(filenameIMU,isrst)

clear IMU OS

dataIMU = dlmread(filenameIMU,'\t',2,0);
nIMU = max(dataIMU(:,1));

order = [];

if nIMU>1
    order = input('\n Order IMUs? [back/sho/elb/wrst] ','s');
end

strspl = strsplit(order,'/');

if nIMU>1
    for ii = 1:nIMU
        IMU(ii).place = strspl{ii};
        IMU(ii).data = dataIMU(dataIMU(:,1)==ii,3:end);
        IMU(ii).time = dataIMU(dataIMU(:,1)==ii,2);
        IMU(ii).ts = timeseries(IMU(ii).data,IMU(ii).time);
    end
end

% h = figure;
% for ii = 1:nIMU
%     plot(IMU(ii).time,IMU(ii).data(:,3))
%     hold on
%     xlabel('Time [s]'); ylabel('Angle [deg]');
% end
% 
% unwrp = input('\n Unwrap IMU angles? (y/n) ','s');
% if strcmp(unwrp,'y')
%     unwrpan = input('\n Unwrap angle? (-180/180) ');
%     for ii = 1:nIMU
%         IMU(ii).data(:,1:3) = unwrap(IMU(ii).data(:,1:3),unwrpan);
%     end
% end
% close(h)

ndata = size(IMU(1).data,2);

if nIMU == 1
    IMU.stime = IMU.time;
    IMU.rl = IMU.data(:,1);
    IMU.pt = IMU.data(:,2);
    IMU.yw = IMU.data(:,3);
    IMU.ori = [IMU.yw,IMU.pt,IMU.rl];
    
    if ndata>3
        IMU.acc = IMU.ests.Data(:,4:6);
        IMU.gyro = rad2deg(IMU.ests.Data(:,7:9));
        IMU.magn = IMU.ests.Data(:,10:12);
        if ndata>12
            IMU.q.q0 = IMU.ests.Data(:,13);
            IMU.q.q1 = IMU.ests.Data(:,14);
            IMU.q.q2 = IMU.ests.Data(:,15);
            IMU.q.q3 = IMU.ests.Data(:,16);
            IMU = EulerfromQuaternion(IMU);
        end
    end
    
elseif nIMU > 1
    
    [IMU(1).sts,IMU(2).sts] = synchronize(IMU(1).ts,IMU(2).ts,'Intersection');

    for ii = 2:nIMU-1
        for jj = 2:nIMU-1
            [IMU(ii-1).sts,IMU(jj+1).sts] = synchronize(IMU(ii-1).sts,IMU(jj+1).ts,'Intersection');
            [IMU(ii).sts,IMU(jj+1).sts] = synchronize(IMU(ii).sts,IMU(jj+1).sts,'Intersection');
        end
    end

    for ii = 1:nIMU
        IMU(ii).stime = IMU(ii).sts.Time;
        IMU(ii).rl = IMU(ii).sts.Data(:,1);
        IMU(ii).pt = IMU(ii).sts.Data(:,2);
        IMU(ii).yw = IMU(ii).sts.Data(:,3);
        IMU(ii).ori = [IMU(ii).rl,IMU(ii).pt,IMU(ii).yw];
        
        if ndata>3
            IMU(ii).acc = IMU(ii).sts.Data(:,4:6);
            IMU(ii).gyro = rad2deg(IMU(ii).sts.Data(:,7:9));
            IMU(ii).magn = IMU(ii).sts.Data(:,10:12);
            if ndata>12
                IMU(ii).q.q0 = IMU(ii).sts.Data(:,13);
                IMU(ii).q.q1 = IMU(ii).sts.Data(:,14);
                IMU(ii).q.q2 = IMU(ii).sts.Data(:,15);
                IMU(ii).q.q3 = IMU(ii).sts.Data(:,16);
                IMU(ii) = EulerfromQuaternion(IMU(ii));
            end
        end
    end
    
    if ~isrst
        for ii = 1:nIMU
            IMU(ii).ori = detrend(IMU(ii).ori,'constant');
            IMU(ii).q.rl = detrend(IMU(ii).q.rl,'constant');
            IMU(ii).q.pt = detrend(IMU(ii).q.pt,'constant');
            IMU(ii).q.yw = detrend(IMU(ii).q.yw,'constant');
        end
    end
    
    nback = find(strcmp({IMU.place}, 'back') == 1);
    nsho = find(strcmp({IMU.place}, 'sho') == 1);
    nelb = find(strcmp({IMU.place}, 'elb') == 1);
    
    OS.time = IMU(1).stime;
    
    OS.shoulder_flexion = IMU(nsho).pt;
    OS.shoulder_adduction = IMU(nsho).rl-IMU(nsho).yw;
    OS.shoulder_rotation = IMU(nsho).yw-IMU(nsho).rl;
    
    OS.elbow_flexion = IMU(nelb).pt+IMU(nsho).pt;
    OS.radial_pronation = IMU(nelb).rl-IMU(nsho).yw+IMU(nsho).rl;
    
end

end
