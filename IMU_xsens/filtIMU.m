function[IMU] = filtIMU(IMU,flow,forder,wname,detaillevel,plt)

if plt
    figure
end

for ii = 1:size(IMU,2)
    %% High pass filtering - high frequency noise removal
    
    [b,a] = butter(forder,flow*2/IMU(ii).fs,'low');
    
    IMU(ii).filt.rl = filtfilt(b,a,IMU(ii).rl);
    IMU(ii).filt.pt = filtfilt(b,a,IMU(ii).pt);
    IMU(ii).filt.yw = filtfilt(b,a,IMU(ii).yw);
    
    IMU(ii).filt.q.rl = filtfilt(b,a,IMU(ii).q.rl);
    IMU(ii).filt.q.pt = filtfilt(b,a,IMU(ii).q.pt);
    IMU(ii).filt.q.yw = filtfilt(b,a,IMU(ii).q.yw);
    
    %% Low pass filtering - drift removal on yaw/pitch (Euler/quat)
    
    decomplevel = wmaxlev(length(IMU(ii).yw),wname);
    [bseline,IMU(ii).filt.yw] = wdriftcorrect(IMU(ii).filt.yw,wname,detaillevel,decomplevel);
    IMU(ii).filt.ori = [IMU(ii).filt.rl,IMU(ii).filt.pt,IMU(ii).filt.yw];
    
    [~,IMU(ii).filt.q.pt] = wdriftcorrect(IMU(ii).filt.q.pt,wname,detaillevel,decomplevel);
    
    % Plot
    if plt
        subplot(3,1,ii)
        plot(IMU(ii).stimem,IMU(ii).yw,'b')
        hold on
        plot(IMU(ii).stimem,IMU(ii).filt.yw)        
        plot(IMU(ii).stimem,bseline,'r','linewidth',1.5)
        legend('Unfiltered','Filtered','Baseline')
        xlabel('Time [min]'); ylabel('Angle [deg]');
        title([IMU(ii).place, ' IMU'])
    end
end
end