%% Analysis IMU data on arm

addpath('txt');

filenameIMU = '20171012_onarm_int.txt';

[IMU,~] = loadsync(filenameIMU);
iselb = 0;

lelb = 16.5*2.54; % [cm] 7.5 to center IMU, 10 to wrist, 16.5 to end index
lsho = 10.5*2.54; % [cm] 7.5 to center IMU, 10.5 to elbow
lenIMU = 4.7; % [cm]
larm = lelb+lsho;

for i = 1:length(IMU(1).stime)
    Xe_sho(:,i) = Rotypr(IMU(1).scth1(i))*[0;-lrsho;0];
    Xe_elb(:,i) = Xe_sho(:,i) + Rotyaw(enc.scth2(i))*[lrelb;0;0];
end

% x as unit vector
x_elb = lelb*cosd(ywe).*cosd(pte);
y_elb = lelb*sind(ywe).*cosd(pte);
z_elb = -lelb*sind(pte);

x_sho = lsho*cosd(yws).*cosd(pts);
y_sho = lsho*sind(yws).*cosd(pts);
z_sho = -lsho*sind(pts);

x_tot = x_elb+x_sho;
y_tot = y_elb+y_sho;
z_tot = z_elb+z_sho;

% Maximum value intervals
thr = 2;
maxtime = timeIMU1(x_tot >= (max(x_tot)-thr));
maxdist = x_tot(x_tot >= (max(x_tot)-thr));

%% Plot distances
figure
subplot(311)
plot(timeIMU1,x_tot)
hold on
plot(timeIMU1,y_tot)
plot(timeIMU1,z_tot)
line(get(gca,'xlim'),[larm larm],'Color','k')
plot(maxtime,maxdist,'*')
title('Total distance')
legend('x','y','z')
subplot(312)
plot(timeIMU1,x_elb)
hold on
plot(timeIMU1,y_elb)
plot(timeIMU1,z_elb)
line(get(gca,'xlim'),[lelb lelb],'Color','k')
title('Elbow distance')
legend('x','y','z')
subplot(313)
plot(timeIMU1,x_sho)
hold on
plot(timeIMU1,y_sho)
plot(timeIMU1,z_sho)
line(get(gca,'xlim'),[lsho lsho],'Color','k')
title('Shoulder distance')
legend('x','y','z')

namespt = strsplit(filenameIMU,'_');
if strcmp(namespt{3},'lat.txt')
    suptitle('Lateral')
elseif strcmp(namespt{3},'sup.txt')
    suptitle('Shoulder Flexion/Extension')
elseif strcmp(namespt{3},'int.txt')
    suptitle('Touch Shoulder')
end

%% Plot angles 
figure
plot(timeIMU1,IMU1)
hold on
plot(timeIMU2,IMU2)
legend('Roll_s','Pitch_s','Yaw_s','Roll_e','Pitch_e','Yaw_e')
xlabel('Time [s]'); ylabel('Angle [deg]');
if strcmp(namespt{3},'lat.txt')
    title('Lateral')
elseif strcmp(namespt{3},'sup.txt')
    title('Shoulder Flexion/Extension')
elseif strcmp(namespt{3},'int.txt')
    title('Touch Shoulder')
end

%% With rotation matrix - NOT
X_sho = [];
X_elb = [];

for i = 1:length(timeIMU1)
    X_sho(:,i) = Rotypr(yws(i),pts(i),rls(i))*[lsho;0;0];
    X_elb(:,i) = X_sho(:,i) + Rotypr(yws(i),pts(i),rls(i))*Rotypr(ywe(i),pte(i),rle(i))*[lelb;0;0];
end

figure 
plot(timeIMU1,X_elb')
line(get(gca,'xlim'),[larm larm],'Color','k')
title('Total distance')
legend('x','y','z')

%% Plot 2D path
figure
subplot(131)
plot(x_tot,y_tot)
xlabel('x'); ylabel('y');
axis equal
subplot(132)
plot(y_tot,z_tot)
xlabel('y'); ylabel('z');
axis equal
subplot(133)
plot(x_tot,z_tot)
xlabel('x'); ylabel('z');
axis equal

