function[IMU] = filtIMU(IMU,plt)
if plt
 figure
end

for ii = 1:size(IMU,2)
    %% High pass filtering - high frequency noise removal
    flow = 4;
    forder = 2;
    
    [b,a] = butter(forder,flow*2/IMU(ii).fs,'low');
    
    IMU(ii).filt.rl = filtfilt(b,a,IMU(ii).rl);
    IMU(ii).filt.pt = filtfilt(b,a,IMU(ii).pt);
    IMU(ii).filt.yw = filtfilt(b,a,IMU(ii).yw);
    
    IMU(ii).filt.q.rl = filtfilt(b,a,IMU(ii).q.rl);
    IMU(ii).filt.q.pt = filtfilt(b,a,IMU(ii).q.pt);
    IMU(ii).filt.q.yw = filtfilt(b,a,IMU(ii).q.yw);
    
    %% Low pass filtering - drift removal on yaw/pitch (Euler/quat)
    wname = 'haar';
    N = 4;
    ord = 14;
    
    [bseline,IMU(ii).filt.yw] = wdriftcorrect(IMU(ii).filt.yw,wname,N,ord);
    IMU(ii).filt.ori = [IMU(ii).filt.rl,IMU(ii).filt.pt,IMU(ii).filt.yw];
    
    [~,IMU(ii).filt.q.pt] = wdriftcorrect(IMU(ii).filt.q.pt,wname,N,ord);
    
    if plt
    % Plot
    subplot(3,1,ii)
    plot(IMU(ii).stimem,IMU(ii).yw,'b')
    hold on
    plot(IMU(ii).stimem,bseline,'r','linewidth',1.5)
    plot(IMU(ii).stimem,IMU(ii).filt.yw)
    xlabel('Time [min]'); ylabel('Angle [deg]');
    title([IMU(ii).place, ' IMU'])
    end
end
end