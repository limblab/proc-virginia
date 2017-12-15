%% Data loading
addpath('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/txt');
filenames = {'20171213_stability_noreset_L1.txt'};
isrst = [1,0];

%%
for  jj = 1:length(filenames)
    
    [IMU,OS] = loadIMU_toOS(filenames{jj},isrst(jj));

    % Plot IMU angles from Euler
    figure
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stime/60,IMU(ii).ori)
        xlabel('Time [s]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
    %suptitle('Euler Angles')
    
    %Plot IMU angles from quaternions
    figure
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stime/60,IMU(ii).q.rl)
        hold on
        plot(IMU(ii).stime/60,IMU(ii).q.pt)
        plot(IMU(ii).stime/60,IMU(ii).q.yw)

        xlabel('Time [s]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
%     suptitle('Quaternions')

end

%% Plotting OS

figure
plot(OS.time,OS.all(:,2:end))
legend(OS.header{2:end})
