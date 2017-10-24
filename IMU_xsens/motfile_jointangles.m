%% Create .mot file with joint angles
motname = '20171012_jointangles.mot';
fid = fopen(motname,'wt');

%% Write on file
nRows = length(IMU(1).stime);
nCols = size(IMU,2)+1;

fprintf(fid,[motname,'\nnRows=%d\nnColumns=%d\n\nUnits are S.I. units (second, meters, Newtons, ...)\nAngles are in degrees.\n\nendheader\n'],nRows,nCols);
fprintf(fid,'time\n');
dlmwrite(motname,[IMU(1).stime,IMU(1).ori,IMU(2).ori],'-append','delimiter','\t','precision','%.6f');
fclose(fid);


