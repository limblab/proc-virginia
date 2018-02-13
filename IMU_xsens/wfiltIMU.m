function[IMU] = wfiltIMU(IMU,wname,detaillevel,plt)
% Filters IMU data for high frequency noise with low pass butterworth
% filter and drift with high pass wavelet filtering

% IMU: IMU data structure
% wname: wavelet type name for drift removal
% detaillevel: order of the wavelet detail coefficients to preserve
% plt: whether to plot filter and unfiltered signals

% Butter low pass filter parameters
flow = 4;
forder = 2;

if plt
    figure
end

for ii = 1:size(IMU,2)
    % Low pass filtering with butter - high frequency noise removal
    [b,a] = butter(forder,flow*2/IMU(ii).fs,'low');
    
    IMU(ii).filt.rl = filtfilt(b,a,IMU(ii).rl);
    IMU(ii).filt.pt = filtfilt(b,a,IMU(ii).pt);
    IMU(ii).filt.yw = filtfilt(b,a,IMU(ii).yw);
    
    if isfield(IMU,'q')
        IMU(ii).filt.q.rl = filtfilt(b,a,IMU(ii).q.rl);
        IMU(ii).filt.q.pt = filtfilt(b,a,IMU(ii).q.pt);
        IMU(ii).filt.q.yw = filtfilt(b,a,IMU(ii).q.yw);
    end
    
    % High pass filtering with wavelets - drift removal on yaw/pitch (Euler/quat)
    decomplevel = wmaxlev(length(IMU(ii).yw),wname);
    [bseline,IMU(ii).filt.yw] = wdriftcorrect(IMU(ii).filt.yw,wname,detaillevel,decomplevel);
    IMU(ii).filt.ori = [IMU(ii).filt.rl,IMU(ii).filt.pt,IMU(ii).filt.yw];
    
    if isfield(IMU,'q')
        [~,IMU(ii).filt.q.pt] = wdriftcorrect(IMU(ii).filt.q.pt,wname,detaillevel,decomplevel);
    end
    
    % Plot
    if plt
        subplot(size(IMU,2),1,ii)
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