%% Create .mot file with joint angles

filenameIMU = '20171012_onarm_lat.txt';
motname = '20171012_jointangles.mot';
fid = fopen(motname,'wt');

data = dlmread(filenameIMU,'\t',2,0);
timeIMU1 = data(data(:,1)==1,2);
timeIMU2 = data(data(:,1)==2,2);
IMU1 = data(data(:,1)==1,3:end);
IMU2 = data(data(:,1)==2,3:end);

ts1 = timeseries(IMU1,timeIMU1);
ts2 = timeseries(IMU2,timeIMU2);
[ts1s,ts2s]=synchronize(ts1,ts2,'Intersection');

timeIMU = ts1s.Time;
IMU1 = ts1s.Data;
IMU2 = ts2s.Data;

%% Write on file
nRows = length(timeIMU);
nCols = size(IMU1,2)+1;
fprintf(fid,[motname,'\nnRows=%d\nnColumns=%d\n\nUnits are S.I. units (second, meters, Newtons, ...)\nAngles are in degrees.\n\nendheader\n'],nRows,nCols);
fprintf(fid,'time\n');
dlmwrite(motname,[timeIMU,IMU1,IMU2],'-append','delimiter','\t');



