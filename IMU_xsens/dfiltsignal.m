function[signalsfilt] = dfiltsignal(signals,IMU,bkptst,plt)

if plt
    figure
end

for ii = 1:size(signals,2)
    for j = 1:size(bkptst,2)
        [~,bkpts(ii,j)] = min(abs(IMU(1).stime-bkptst(ii,j)));
    end
    
    signalsfilt(:,ii) = detrend(signals(:,ii),'linear',bkpts(ii,:))+ mean(signals(1:bkpts(ii,1)),ii);
    
    if plt
        subplot(size(signals,2),1,ii)
        plot(IMU(ii).stime,signalsfilt(:,ii),'b')
        hold on
        plot(IMU(ii).stime,signals(:,ii))
        legend('Unfiltered','Filtered')
        xlabel('Time [min]'); ylabel('Angle [deg]');
    end
    
end
end
        
