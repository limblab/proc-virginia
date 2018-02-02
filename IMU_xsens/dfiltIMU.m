function[IMU] = dfiltIMU(IMU,flow,forder,bkpts,plt)

if plt
    figure
end

for ii = 1:size(IMU,2)
    %% High pass filtering with butter - high frequency noise removal
    
    [b,a] = butter(forder,flow*2/IMU(ii).fs,'low');
    
    IMU(ii).filt.rl = filtfilt(b,a,IMU(ii).rl);
    IMU(ii).filt.pt = filtfilt(b,a,IMU(ii).pt);
    IMU(ii).filt.yw = filtfilt(b,a,IMU(ii).yw);
    
    if isfield(IMU,'q')
        IMU(ii).filt.q.rl = filtfilt(b,a,IMU(ii).q.rl);
        IMU(ii).filt.q.pt = filtfilt(b,a,IMU(ii).q.pt);
        IMU(ii).filt.q.yw = filtfilt(b,a,IMU(ii).q.yw);
    end
    
    %% Low pass filtering with detrend - drift removal on yaw/pitch (Euler/quat)
    
    IMU(ii).filt.yw = detrend(IMU(ii).filt.yw,'linear',bkpts(ii,:));
    IMU(ii).filt.ori = [IMU(ii).filt.rl,IMU(ii).filt.pt,IMU(ii).filt.yw];
    
    if isfield(IMU,'q')
        IMU(ii).filt.q.pt = detrend(IMU(ii).filt.q.pt,'linear',bkpts(ii,:));
    end
    
    %% Plot
    if plt
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).yw,'b')
        hold on
        plot(IMU(ii).stimem,IMU(ii).filt.yw)
        legend('Unfiltered','Filtered')
        xlabel('Time [min]'); ylabel('Angle [deg]');
        title([IMU(ii).place, ' IMU'])
    end
end
end