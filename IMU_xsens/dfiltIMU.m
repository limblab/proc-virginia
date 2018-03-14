function[IMU] = dfiltIMU(IMU,bkptst,IMUfilt,plt)
% Filters IMU data for high frequency noise with low pass butterworth
% filter and drift with detrend MATLAB command

% IMU: IMU data structure
% bkptst: matrix with time points where there is a change in trend in the
% signal. Each row corresponds to a different IMU and each column to a
% different time point
% plt: whether to plot filter and unfiltered signals

% Butter low pass filter parameters
flow = 8;
forder = 5;

if plt
    figure
end

for ii = 1:size(IMU,2)
    for j = 1:size(bkptst,2)
        [~,bkpts(ii,j)] = min(abs(IMU(1).stimem-bkptst(ii,j)));
    end
end

for ii = 1:size(IMU,2)
    % Low pass filtering with butter - high frequency noise removal
%     [b,a] = butter(forder,flow*2/IMU(ii).fs,'low');
%     
%     IMU(ii).filt.rl = filtfilt(b,a,IMU(ii).rl);
%     IMU(ii).filt.pt = filtfilt(b,a,IMU(ii).pt);
%     IMU(ii).filt.yw = filtfilt(b,a,IMU(ii).yw);
%     
%     if isfield(IMU,'q')
%         IMU(ii).filt.q.rl = filtfilt(b,a,IMU(ii).q.rl);
%         IMU(ii).filt.q.pt = filtfilt(b,a,IMU(ii).q.pt);
%         IMU(ii).filt.q.yw = filtfilt(b,a,IMU(ii).q.yw);
%     end
    
    IMU(ii).filt.rl = IMU(ii).rl;
    IMU(ii).filt.pt = IMU(ii).pt;
    IMU(ii).filt.yw = IMU(ii).yw;
    
    IMU(ii).filt.q.rl = IMU(ii).q.rl;
    IMU(ii).filt.q.pt = IMU(ii).q.pt;
    IMU(ii).filt.q.yw = IMU(ii).q.yw;
    
if IMUfilt(ii) == 1
    % High pass filtering with detrend - drift removal on yaw/pitch (Euler/quat)  
    IMU(ii).filt.yw = detrend(IMU(ii).filt.yw,'linear',bkpts(ii,:))+ mean(IMU(ii).filt.yw(1:bkpts(ii,1)));
    IMU(ii).filt.ori = [IMU(ii).filt.rl,IMU(ii).filt.pt,IMU(ii).filt.yw];
    
    if isfield(IMU,'q')
        IMU(ii).filt.q.pt = detrend(IMU(ii).filt.q.pt,'linear',bkpts(ii,:))+ mean(IMU(ii).filt.q.pt(1:bkpts(ii,1)));
        IMU(ii).filt.q.rl = detrend(IMU(ii).filt.q.rl,'linear',bkpts(ii,:))+ mean(IMU(ii).filt.q.rl(1:bkpts(ii,1)));
        IMU(ii).filt.q.yw = detrend(IMU(ii).filt.q.yw,'linear',bkpts(ii,:))+ mean(IMU(ii).filt.q.yw(1:bkpts(ii,1)));
    end
end
    % Plot
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