%% Create .mot file with joint angles
motname = '20171012_jointangles.mot';
delete motname
fid = fopen(motname,'wt');

%% Write on file
nRows = length(IMU(1).stime);
nCols = size(IMU,2)+1;

fprintf(fid,[motname,'\nnRows=%d\nnColumns=%d\n\nUnits are S.I. units (second, meters, Newtons, ...)\nAngles are in degrees.\n\nendheader\n'],nRows,nCols);
fprintf(fid,'%s\t %s\t %s\t %s\t %s\t %s\n',header{:});
dlmwrite(motname,OS.all,'-append','delimiter','\t','precision','%.6f');
fclose(fid);

%% Get OpenSim angles
clear OS

OS.time = IMU(1).stime;

OS.shoulder_flexion = IMU(1).yw;
OS.shoulder_adduction = IMU(1).pt;
OS.shoulder_rotation = IMU(1).rl;

OS.elbow_flexion = IMU(2).yw-IMU(1).yw;
OS.radial_pronation = IMU(2).rl-IMU(1).pt;

header = fieldnames(OS);

OS.all = [];
for ii = 1:length(header)
    OS.all = [OS.all OS.(header{ii})];
end
