%% Analysis IMU data on arm
addpath('txt');

filenameIMU = '20171026_onarm_angid.txt';

[IMU,~] = loadsync(filenameIMU);
iselb = 0;

%% Position IMU 
lelb = 16.5*2.54; % [cm] 7.5 to center IMU, 10 to wrist, 16.5 to end index
lsho = 10.5*2.54; % [cm] 7.5 to center IMU, 10.5 to elbow
lenIMU = 4.7; % [cm]
larm = lelb+lsho;

stime = IMU(1).stime;

X_sho = [];
X_elb = [];

for i = 1:length(IMU(1).stime)
    X_sho(i,:) = Rotypr(IMU(1).yw(i),IMU(1).pt(i),IMU(1).rl(i))*[lsho;0;0];
    X_elb(i,:) = X_sho(i,:)' + Rotypr(IMU(2).yw(i),IMU(2).pt(i),IMU(2).rl(i))*[lelb;0;0];
end

%% x as unit vector
x_elb = X_elb(:,1);
y_elb = X_elb(:,2);
z_elb = X_elb(:,3);

x_sho = X_sho(:,1);
y_sho = X_sho(:,2);
z_sho = X_sho(:,3);

% Maximum value intervals
thr = 2;
maxtime = IMU(1).stime(x_elb >= (max(x_elb)-thr));
maxdist = x_elb(x_elb >= (max(x_elb)-thr));

%% Plot distances
figure
subplot(211)
plot(stime,x_elb)
hold on
plot(stime,y_elb)
plot(stime,z_elb)
line(get(gca,'xlim'),[larm larm],'Color','k')
title('Elbow distance')
legend('x','y','z')
subplot(212)
plot(stime,x_sho)
hold on
plot(stime,y_sho)
plot(stime,z_sho)
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
plot(stime,IMU(1).sts.Data(:,1:3))
hold on
plot(stime,IMU(2).sts.Data(:,1:3))
legend('Roll_s','Pitch_s','Yaw_s','Roll_e','Pitch_e','Yaw_e')
xlabel('Time [s]'); ylabel('Angle [deg]');
if strcmp(namespt{3},'lat.txt')
    title('Lateral')
elseif strcmp(namespt{3},'sup.txt')
    title('Shoulder Flexion/Extension')
elseif strcmp(namespt{3},'int.txt')
    title('Touch Shoulder')
end

%% Plot 2D path
figure
subplot(221)
plot(x_elb,y_elb)
xlabel('x'); ylabel('y');
axis equal; grid on
subplot(222)
plot(y_elb,z_elb)
xlabel('y'); ylabel('z');
axis equal; grid on
subplot(223)
plot(x_elb,z_elb)
xlabel('x'); ylabel('z');
axis equal; grid on
subplot(224)
plot3(x_elb,y_elb,z_elb)
xlabel('x'); ylabel('y'); zlabel('z');
axis equal; grid on

%% Get OpenSim angles
clear OS

OS.Eul.time = IMU(1).stime;

OS.Eul.shoulder_flexion = IMU(1).pt;
OS.Eul.shoulder_adduction = IMU(1).yw;
OS.Eul.shoulder_rotation = IMU(1).rl;

OS.Eul.elbow_flexion = IMU(2).pt-IMU(1).pt;
OS.Eul.radial_pronation = IMU(2).rl-IMU(1).rl;

header = fieldnames(OS.Eul);

OS.Eul.all = [];
for ii = 1:length(header)
    OS.Eul.all = [OS.Eul.all OS.Eul.(header{ii})];
end

figure
plot(OS.Eul.time,OS.Eul.all(:,2:end))
legend(header{2:end})

%% Quaternions 
figure
plot(stime,[IMU(1).q.rl,IMU(1).q.pt,IMU(1).q.yw])
hold on
plot(stime,[IMU(2).q.rl,IMU(2).q.pt,IMU(2).q.yw])
legend('Roll_s','Pitch_s','Yaw_s','Roll_e','Pitch_e','Yaw_e')
xlabel('Time [s]'); ylabel('Angle [deg]');

OS.Quat.time = IMU(1).stime;

OS.Quat.shoulder_flexion = IMU(1).q.yw;
OS.Quat.shoulder_adduction = IMU(1).q.pt;
OS.Quat.shoulder_rotation = IMU(1).q.rl;

OS.Quat.elbow_flexion = IMU(2).q.yw-IMU(1).q.yw;
OS.Quat.radial_pronation = IMU(2).q.rl-IMU(1).q.rl;

OS.Quat.all = [];
for ii = 1:length(header)
    OS.Quat.all = [OS.Quat.all OS.Quat.(header{ii})];
end

figure
plot(OS.Quat.time,OS.Quat.all(:,2:end))
legend(header{2:end})

