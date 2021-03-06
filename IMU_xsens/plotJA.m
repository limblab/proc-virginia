function[] = plotJA(IMU,JA,filenames,varargin)

if isempty(varargin)
    opts = {'joint','body','joint diff'};%,'joint calib','body calib'};
else
    opts = varargin{1};
end

if any(strcmp(opts,'joint'))
    figure('name',[filenames, '-Joint Angles'])
    for ii = 1:size(JA,2)-1
        subplot(size(JA,2)-1,1,ii)
        plot(JA(1).time,(JA(ii).rl))
        hold on
        if ~strcmp(JA(ii).place,'elb')
            plot(JA(1).time,(JA(ii).pt))
            plot(JA(1).time,unwrap(JA(ii).yw))
        else
            plot(JA(1).time,(JA(ii).yw))
        end
        xlabel('Time [min]'); ylabel('Angle [deg]');
        if strcmp(JA(ii).place,'elb')
            legend('FE','PS/R')
        elseif strcmp(JA(ii).place,'sho')
            legend('FE','PS/R','AA')
        end
        title([JA(ii).place, ' joint'])
        set(gca,'Fontsize',12)
    end
end

if any(strcmp(opts,'joint diff'))
    figure('name',[filenames, '-Joint Angles diff'])
    for ii = 1:size(JA,2)-1
        subplot(size(JA,2)-1,1,1)
        plot(JA(1).time,(JA(ii).rld))
        hold on
        plot(JA(1).time,(JA(ii).ptd))
        plot(JA(1).time,unwrap(JA(ii).ywd))
        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll/FE','Pitch/PS/R','Yaw/AA')
        title([JA(ii).place, ' joint'])
    end
end

if any(strcmp(opts,'body'))
    figure('name',[filenames, '-Body IMU Angles'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,(JA(ii).rlg))
        hold on
        plot(IMU(ii).stimem,(JA(ii).ptg))
        plot(IMU(ii).stimem,(JA(ii).ywg))
        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
end
% if any(strcmp(opts,'joint calib'))
%     figure('name',[filenames, '-Joint Angles from Calibration'])
%     for ii = 1:size(IMU,2)-1
%         subplot(size(IMU,2),1,ii)
%         plot(IMU(ii).timem_calib,(JA(ii).rlc))
%         hold on
%         plot(IMU(ii).timem_calib,(JA(ii).ptc))
%         plot(IMU(ii).timem_calib,(JA(ii).ywc))
%         xlabel('Time [min]'); ylabel('Angle [deg]');
%         legend('Roll','Pitch','Yaw')
%         title([IMU(ii).place, ' IMU'])
%     end
% end
% if any(strcmp(opts,'body calib'))
%     figure('name',[filenames, '-Body IMU Angles from Calibration'])
%     for ii = 1:size(IMU,2)
%         subplot(size(IMU,2),1,ii)
%         plot(IMU(ii).timem_calib,(JA(ii).rlgc))
%         hold on
%         plot(IMU(ii).timem_calib,(JA(ii).ptgc))
%         plot(IMU(ii).timem_calib,(JA(ii).ywgc))
%         xlabel('Time [min]'); ylabel('Angle [deg]');
%         legend('Roll','Pitch','Yaw')
%         title([IMU(ii).place, ' IMU'])
%     end
% end

end