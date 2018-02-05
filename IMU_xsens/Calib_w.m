%% Calibration with IMU angular velocity
[val,stix] = min(abs(IMU(1).stimem-0.05));
[val,dyix] = min(abs(IMU(1).stimem-0.274));

v1 = mean(IMU(1).acc(2:stix,:));
v1n = v1/norm(v1);

v2 = (IMU(1).gyro(dyix,:));
v2n = v2/norm(v2);

v3 = cross(v2n,v1n);
v3n = v3/norm(v3);

v4n = cross(v3n,v1n);

v = [v1n;v4n;v3n];

v1v = [0, 0, 0; v1n];
v2v = [0, 0, 0; v2n];
v3v = [0, 0, 0; v3n];
v4v = [0, 0, 0; v4n];

figure
plot3(v1v(:,1),v1v(:,2),v1v(:,3),'r')
grid on
hold on
plot3(v4v(:,1),v4v(:,2),v4v(:,3),'b')
plot3(v3v(:,1),v3v(:,2),v3v(:,3),'g')

%% Plot calibrated data
cdata = [];
cdataq = [];

for ii = 1: length(IMU.stime)
    cdata(ii,:) = v*IMU.data(ii,1:3)';
    cdataq(ii,:) = v*IMU.q.ori(ii,:)';
    
end

figure
subplot(1,2,1)
plot(IMU.stime-IMU.stime(1),cdata)
legend('Roll','Pitch','Yaw'); title('Euler')
subplot(1,2,2)
plot(IMU.stime-IMU.stime(1),cdataq)
legend('Roll','Pitch','Yaw'); title('Quaternion')