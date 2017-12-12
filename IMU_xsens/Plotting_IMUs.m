%% Data loading
filenames = {'20171208_stability_reset.txt','20171208_stability_noreset.txt'};
isrst = [1,0];

%%
for  jj = 1:length(filenames)
    
    [IMU,OS] = loadIMU_toOS(filenames{jj},isrst(jj));

    % Plot IMU angles from Euler
%     figure
%     for ii = 2:size(IMU,2)
%         subplot(size(IMU,2)-1,1,ii-1)
%         plot(IMU(ii).stime-IMU(ii).stime(1),IMU(ii).ori)
%         xlabel('Time [s]'); ylabel('Angle [deg]');
%         legend('Roll','Pitch','Yaw')
%         title([IMU(ii).place, ' IMU'])
%     end
    %suptitle('Euler Angles')
    
    %Plot IMU angles from quaternions
    figure
    for ii = 2:size(IMU,2)
        subplot(size(IMU,2)-1,1,ii-1)
        plot(IMU(ii).stime-IMU(ii).stime(1),IMU(ii).q.rl)
        hold on
        plot(IMU(ii).stime-IMU(ii).stime(1),IMU(ii).q.pt)
        plot(IMU(ii).stime-IMU(ii).stime(1),IMU(ii).q.yw)

        xlabel('Time [s]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
%     suptitle('Quaternions')

end
