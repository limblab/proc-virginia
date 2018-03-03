%% Create .mot file with joint angles
IMUname = strsplit(filenames{2},'.');
motname = [IMUname{1},'_JA_J.mot'];
%txtpath = 'C:\Users\vct1641\Documents\Data\data-IMU\mot\';
fid = fopen(fullfile([txtpath,motname]),'wt'); %txtpath is loaded in first section of Plotting_IMUs

%% Write on file
nRows = length(IMU(1).stime);
nCols = length(header);

fprintf(fid,[motname,'\nnRows=%d\nnColumns=%d\n\nUnits are S.I. units (second, meters, Newtons, ...)\nAngles are in degrees.\n\nendheader\n'],nRows,nCols);
fprintf(fid,'%s\t %s\t %s\t %s\t %s\t %s\n',header{:});
dlmwrite([txtpath,motname],OpenSim.all,'-append','delimiter','\t','precision','%.6f');
fclose(fid);

%% Get OpenSim angles from calibration - Monkey model
clear OpenSim
nsho = find(strcmp({JA.place},'sho'));
nelb = find(strcmp({JA.place},'elb'));

OpenSim.time = IMU(1).stime;

OpenSim.shoulder_rotation = -(JA(nsho).yw)';
OpenSim.shoulder_flexion = -JA(nsho).rl';
OpenSim.shoulder_adduction = -(JA(nsho).pt)';

% OpenSim.shoulder_rotation = (JA(nsho).ywg)';
% OpenSim.shoulder_flexion = JA(nsho).rlg';
% OpenSim.shoulder_adduction = -(JA(nsho).ptg)';

OpenSim.elbow_flexion = -JA(nelb).rl'+20;
OpenSim.radial_pronation = -(JA(nelb).yw');

header = fieldnames(OpenSim);

OpenSim.all = [];
for ii = 1:length(header)
    OpenSim.all = [OpenSim.all OpenSim.(header{ii})];
end

figure
plot(OpenSim.time,OpenSim.all(:,2:end))
legend(header{2:end})
xlabel('Time [s]'); ylabel('Angles [deg]');

%% Get OpenSim angles from calibration - Human model
clear OS

OpenSim.time = IMU(1).stime-IMU(1).stime(1);

OpenSim.elv_angle = -unwrap(JA(2).yw)'+90;
OpenSim.shoulder_elv = -JA(2).rl';
OpenSim.shoulder_rot = JA(2).pt';

OpenSim.elbow_flexion = JA(1).rl'+90+10;
OpenSim.pro_sup = (JA(1).yw');

header = fieldnames(OpenSim);

OpenSim.all = [];
for ii = 1:length(header)
    OpenSim.all = [OpenSim.all OpenSim.(header{ii})];
end

figure
plot(OpenSim.time,OpenSim.all(:,2:end))
legend(header{2:end})

%% Get OpenSim angles from difference
clear OpenSim

OpenSim.time = IMU(1).stime;

% OS.shoulder_flexion = IMU(1).yw;
% OS.shoulder_adduction = IMU(1).pt;
% OS.shoulder_rotation = IMU(1).rl;
% 
% OS.elbow_flexion = IMU(2).yw-IMU(1).yw;
% OS.radial_pronation = IMU(2).rl-IMU(1).pt;

OpenSim.shoulder_flexion = IMU(1).pt;
OpenSim.shoulder_adduction = IMU(1).rl-IMU(1).yw;
OpenSim.shoulder_rotation = IMU(1).yw-IMU(1).rl;

OpenSim.elbow_flexion = IMU(2).pt+IMU(1).pt;
OpenSim.radial_pronation = IMU(2).rl-IMU(1).yw+IMU(1).rl;

header = fieldnames(OpenSim);

OpenSim.all = [];
for ii = 1:length(header)
    OpenSim.all = [OpenSim.all OpenSim.(header{ii})];
end

figure
plot(OpenSim.time,OpenSim.all(:,2:end))
legend(header{2:end})
