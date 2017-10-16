%% Analysis IMU data on arm
addpath('txt');
filenameIMU = '20171011_onarm.txt';

data = dlmread(filenameIMU,'\t',2,0);
timeIMU1 = data(data(:,1)==1,2);
timeIMU2 = data(data(:,1)==2,2);
IMU1 = data(data(:,1)==1,3:end);
IMU2 = data(data(:,1)==2,3:end);

lelb = 7.5*2.54; % [cm] 7.5 to center IMU, 10 to wrist, 16.5 to end index
lsho = []; % [cm] 7.5 to center IMU, 10.5 to elbow
lenIMU = 4.7; % [cm]

is2D = 1;

if is2D == 1
    rpy = [0 0 1];
else
    rpy = [1 0 1];
end

rle = IMU1(:,1);
pte = IMU1(:,2);
ywe = IMU1(:,3);

rls = IMU2(:,1);
pts = IMU2(:,2);
yws = IMU2(:,3);

rpymat = repmat(rpy,length(IMU1),1);
rpymat_elb = rpymat*lelb;

% x as unit vector
x_elb = lelb*cosd(ywe).*cosd(pte);
y_elb = lelb*sind(ywe).*cosd(pte);
z_elb = lelb*sind(rle);

x_sho = lsho*cosd(yws).*cosd(pts);
y_sho = lsho*sind(yws).*cosd(pts);
z_sho = lsho*sind(rls);

x_tot = x_elb+x_sho;
y_tot = y_elb+y_sho;
z_tot = z_elb+z_sho;

% y as unit vector
% x_elb = lenelb*(-cosd(yw).*sind(pt).*sind(rl)-sind(yw).*cosd(rl));
% y_elb = lenelb*(-sind(yw).*sind(pt).*sind(rl)+cosd(yw).*cosd(rl));
% z_elb = lenelb*cosd(pt).*sind(rl);

figure
%subplot(211)
plot(timeIMU1,x_elb)
hold on
plot(timeIMU1,y_elb)
%plot(timeIMU1,z_elb)
legend('x','y')
% subplot(212)
% plot(timeIMU1,dist_elb)
% legend('Roll','Pitch','Yaw')

%% Maximum value intervals

thr = 0.1;
maxtime = timeIMU1(x_elb >= (max(x_elb)-thr));
maxdist = x_elb(x_elb >= (max(x_elb)-thr));

figure
plot(timeIMU1,x_elb)
hold on
plot(timeIMU1,y_elb)
plot(maxtime,maxdist,'*')







