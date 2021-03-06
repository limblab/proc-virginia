function[] = plotIMU(IMU,filenames,varargin)

if isempty(varargin)
    opts = {'eul','quat','acc','gyro','magn','nmagn','acc_calib','eul_calib'};
else
    opts = varargin{1};
end

if any(strcmp(opts,'eul')) && isfield(IMU,'ori')
    % Plot IMU angles from Euler
    figure('name',[filenames, '-Euler'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).ori)
        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        set(gca,'Fontsize',12);
        title([IMU(ii).place, ' IMU'],'Fontsize',12)
    end
end
if any(strcmp(opts,'eul_calib')) && isfield(IMU,'eul_calib')
    % Plot IMU angles from Euler
    figure('name',[filenames, '-Euler from Calibration'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).timem_calib,IMU(ii).eul_calib)
        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        set(gca,'Fontsize',12);
        title([IMU(ii).place, ' IMU'],'Fontsize',12)
    end
end
if any(strcmp(opts,'quat')) && isfield(IMU,'q')
    % Plot IMU angles from quaternions
    figure('name',[filenames, '-Quaternions'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).q.rl)
        hold on
        plot(IMU(ii).stimem,IMU(ii).q.pt)
        plot(IMU(ii).stimem,IMU(ii).q.yw)
        xlabel('Time [min]'); ylabel('Angle [deg]');
        legend('Roll','Pitch','Yaw')
        title([IMU(ii).place, ' IMU'])
    end
end
if any(strcmp(opts,'acc')) && isfield(IMU,'acc')
    % Plot accelerations
    figure('name',[filenames '-Accelerations'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).acc)
        xlabel('Time [min]'); ylabel('Acceleration [m/s^2]');
        legend('a_x','a_y','a_z')
        title([IMU(ii).place, ' IMU'])
    end
end
if any(strcmp(opts,'acc_calib')) && isfield(IMU,'acc_calib')
    % Plot accelerations
    figure('name',[filenames '-Accelerations from Calibration'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).timem_calib,IMU(ii).acc_calib)
        xlabel('Time [min]'); ylabel('Acceleration [m/s^2]');
        legend('a_x','a_y','a_z')
        title([IMU(ii).place, ' IMU'])
    end
end
if any(strcmp(opts,'gyro')) && isfield(IMU,'gyro')
    % Plot angular velocity
    figure('name',[filenames '-Gyroscope'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).gyro)
        xlabel('Time [min]'); ylabel('Angular Velocity [deg/s]');
        legend('w_x','w_y','w_z')
        title([IMU(ii).place, ' IMU'])
    end
end
if any(strcmp(opts,'magn')) && isfield(IMU,'magn')
    % Plot magnetic field
    figure('name',[filenames '-Magnetic Field'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).magn)
        xlabel('Time [min]'); ylabel('Magnetic Field [-]');
        legend('m_x','m_y','m_z')
        title([IMU(ii).place, ' IMU'])
    end
end
if any(strcmp(opts,'nmagn')) && isfield(IMU,'nmagn')
    % Plot normalized magnetic field - should be close to 1
    figure('name',[filenames '-Normalized Magnetic Field'])
    for ii = 1:size(IMU,2)
        subplot(size(IMU,2),1,ii)
        plot(IMU(ii).stimem,IMU(ii).nmagn)
        xlabel('Time [min]'); ylabel('Normalized Magnetic Field [-]');
        title([IMU(ii).place, ' IMU'])
    end
end
end
