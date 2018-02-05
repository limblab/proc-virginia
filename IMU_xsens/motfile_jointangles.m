%% Create .mot file with joint angles
IMUname = strsplit(filenames{:},'.');
motname = [IMUname{1},'_JA.mot'];
fid = fopen([txtpath,'/',motname],'wt');

%% Write on file
nRows = length(IMU(1).stime);
nCols = length(header);

fprintf(fid,[motname,'\nnRows=%d\nnColumns=%d\n\nUnits are S.I. units (second, meters, Newtons, ...)\nAngles are in degrees.\n\nendheader\n'],nRows,nCols);
fprintf(fid,'%s\t %s\t %s\t %s\t %s\t %s\n',header{:});
dlmwrite([txtpath,'/',motname],OS.all,'-append','delimiter','\t','precision','%.6f');
fclose(fid);

%% Get OpenSim angles
clear OS

OS.time = IMU(1).stime-IMU(1).stime(1);

OS.elv_angle = -unwrap(JA(2).yw)'+90;
OS.shoulder_elv = -JA(2).rl';
OS.shoulder_rot = JA(2).pt';

OS.elbow_flexion = JA(1).rl'+90+20;
OS.pro_sup = (JA(1).yw');

header = fieldnames(OS);

OS.all = [];
for ii = 1:length(header)
    OS.all = [OS.all OS.(header{ii})];
end

figure
plot(OS.time,OS.all(:,2:end))
legend(header{2:end})

%% Get OpenSim angles
clear OS

OS.time = IMU(1).stime-IMU(1).stime(1);

% OS.shoulder_flexion = IMU(1).yw;
% OS.shoulder_adduction = IMU(1).pt;
% OS.shoulder_rotation = IMU(1).rl;
% 
% OS.elbow_flexion = IMU(2).yw-IMU(1).yw;
% OS.radial_pronation = IMU(2).rl-IMU(1).pt;

OS.shoulder_flexion = IMU(1).pt;
OS.shoulder_adduction = IMU(1).rl-IMU(1).yw;
OS.shoulder_rotation = IMU(1).yw-IMU(1).rl;

OS.elbow_flexion = IMU(2).pt+IMU(1).pt;
OS.radial_pronation = IMU(2).rl-IMU(1).yw+IMU(1).rl;

header = fieldnames(OS);

OS.all = [];
for ii = 1:length(header)
    OS.all = [OS.all OS.(header{ii})];
end

figure
plot(OS.time,OS.all(:,2:end))
legend(header{2:end})
