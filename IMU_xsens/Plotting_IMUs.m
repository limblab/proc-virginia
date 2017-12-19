%% Data loading
addpath('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt');
filenames = {'20171214_stability_noreset_L1_nometal_2.txt'};
isrst = [0,0];

%%
for  jj = 1:length(filenames)
    
    [IMU,OS] = loadIMU_toOS(filenames{jj},isrst(jj));
    %IMU = IMU_nometal2;
    % Plot IMU angles from Euler
    figure('name',filenames{jj})
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).ori)
        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
    %suptitle('Euler Angles')
    
    %Plot IMU angles from quaternions
%     figure
%     for ii = 1:size(IMU,2)
%         subplot(size(IMU,2),1,ii)
%         plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).q.rl)
%         hold on
%         plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).q.pt)
%         plot((IMU(ii).stime-IMU(ii).time(1))/60,IMU(ii).q.yw)
% 
%         xlabel('Time [min]'); ylabel('Angle [deg]');
%         legend('Roll','Pitch','Yaw')
%         title([IMU(ii).place, ' IMU'])
%     end
%     suptitle('Quaternions')

end
%% Plotting IMU non detrend
figure
for ii = 1:size(IMU,2)
    subplot(size(IMU,2),1,ii)
    plot(IMU(ii).stime-IMU(ii).stime(1),IMU(ii).sts.Data(:,1:3))
    xlabel('Time [s]'); ylabel('Angle [deg]');
    legend('Roll','Pitch','Yaw')
    title([IMU(ii).place, ' IMU'])
end

%% Plotting OS

figure
plot(OS.time,OS.all(:,2:end))
legend(OS.header{2:end})
