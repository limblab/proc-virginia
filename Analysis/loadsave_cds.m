%% Set up
meta.lab=6;
meta.ranBy='Virginia';
meta.monkey='Han';
meta.date='20180228';
meta.task='COC3D'; % for the loading of cds
meta.taskAlias={'COC3D_001','COC3D_002'}; % for the filename (cell array list for files to load and save)
meta.array='LeftS1Area2'; % for the loading of cds
meta.arrayAlias='area2'; % for the filename
meta.project='COC3D'; % for the folder in data-preproc
meta.superfolder=fullfile('C:\Users\vct1641\Documents\Data\data-preproc\',meta.project,meta.monkey); % folder for data dump
meta.folder=fullfile(meta.superfolder,meta.date); % compose subfolder and superfolder

meta.neuralPrefix = [meta.monkey '_' meta.date '_' meta.arrayAlias];
meta.IMUPrefix = [meta.monkey '_' meta.date '_IMU_'];

EMGextrafile = 1;

if strcmp(meta.monkey,'Chips')
    meta.mapfile='C:\Users\vct1641\Documents\Data\data-preproc\Meta\Mapfiles\Chips\left_S1\SN 6251-001455.cmp';
elseif strcmp(meta.monkey,'Han')
    meta.mapfile='C:\Users\vct1641\Documents\Data\data-preproc\Meta\Mapfiles\Han\left_S1\SN 6251-001459.cmp';
    if EMGextrafile == 1
        altMeta = meta;
        altMeta.array='';
        altMeta.arrayAlias='EMGextra';
        altMeta.neuralPrefix = [altMeta.monkey '_' altMeta.date '_' altMeta.arrayAlias];
        % altMeta.mapfile=???;
    end
elseif strcmp(meta.monkey,'Lando')
    warning('mapfiles not found for Lando yet')
%     meta.mapfile='C:\Users\rhc307\Projects\limblab\data-preproc\Meta\Mapfiles\Chips\left_S1\SN 6251-001455.cmp';
    altMeta = meta;
    altMeta.array='RightCuneate';
    altMeta.arrayAlias='cuneate';
    altMeta.neuralPrefix = [altMeta.monkey '_' altMeta.date '_' altMeta.arrayAlias];
%     altMeta.mapfile=???;
end

%% Move data into subfolder
if ~exist(meta.folder,'dir')
    mkdir(meta.folder)
    movefile(fullfile(meta.superfolder,[meta.monkey '_' meta.date '*']),meta.folder)
end

%% Set up folder structure
if ~exist(fullfile(meta.folder,'preCDS'),'dir')
    mkdir(fullfile(meta.folder,'preCDS'))
    movefile(fullfile(meta.folder,[meta.neuralPrefix '*']),fullfile(meta.folder,'preCDS'))
    if exist('altMeta','var')
        movefile(fullfile(meta.folder,[altMeta.neuralPrefix '*']),fullfile(meta.folder,'preCDS'))
    end
end
if ~exist(fullfile(meta.folder,'preCDS','merging'),'dir')
    mkdir(fullfile(meta.folder,'preCDS','merging'))
    movefile(fullfile(meta.folder,'preCDS',[meta.neuralPrefix '*.nev']),fullfile(meta.folder,'preCDS','merging'))
    if exist('altMeta','var') && ~isempty(altMeta.array)
        movefile(fullfile(meta.folder,'preCDS',[altMeta.neuralPrefix '*.nev']),fullfile(meta.folder,'preCDS','merging'))
    end
end
if ~exist(fullfile(meta.folder,'preCDS','Final'),'dir')
    mkdir(fullfile(meta.folder,'preCDS','Final'))
    movefile(fullfile(meta.folder,'preCDS',[meta.neuralPrefix '*.n*']),fullfile(meta.folder,'preCDS','Final'))
    if exist('altMeta','var')
        movefile(fullfile(meta.folder,'preCDS',[altMeta.neuralPrefix '*.n*']),fullfile(meta.folder,'preCDS','Final'))
    end
end

if exist([meta.folder,'*_colorTracking_*.mat'],'file')
    if ~exist(fullfile(meta.folder,'ColorTracking'),'dir')
        mkdir(fullfile(meta.folder,'ColorTracking'))
        movefile(fullfile(meta.folder,'*_colorTracking_*.mat'),fullfile(meta.folder,'ColorTracking'))
    end
    if ~exist(fullfile(meta.folder,'ColorTracking','Markers'),'dir')
        mkdir(fullfile(meta.folder,'ColorTracking','Markers'))
    end
end

if ~exist(fullfile(meta.folder,'IMU'),'dir')
    mkdir(fullfile(meta.folder,'IMU'))
    movefile(fullfile(meta.folder,'*_IMU_*.txt'),fullfile(meta.folder,'IMU'))
end

if ~exist(fullfile(meta.folder,'OpenSim'),'dir')
    mkdir(fullfile(meta.folder,'OpenSim'))
end
if ~exist(fullfile(meta.folder,'CDS'),'dir')
    mkdir(fullfile(meta.folder,'CDS'))
end
if ~exist(fullfile(meta.folder,'TD'),'dir')
    mkdir(fullfile(meta.folder,'TD'))
end

%% Merge and strip files for spike sorting
% Run processSpikesForSorting for the first time to combine spike data from
% all files with a name starting with file_prefix.
processSpikesForSorting(fullfile(meta.folder,'preCDS','merging'),meta.neuralPrefix);
if exist('altMeta','var') && ~isempty(altMeta.array)
    processSpikesForSorting(fullfile(altMeta.folder,'preCDS','merging'),altMeta.neuralPrefix);
end

% Now sort in Offline Sorter!

%% Load colorTracking file (and settings if desired) -- NOTE: Can do this simultaneously with sorting, since it takes some time
first_time = 1;
for fileIdx = 1:length(meta.taskAlias)
    colorTrackingFilename = [meta.monkey '_' meta.date '_colorTracking_' meta.taskAlias{fileIdx}];

    fname_load=ls(fullfile(meta.folder,'ColorTracking',[colorTrackingFilename '*']));
    load(deblank(fullfile(meta.folder,'ColorTracking',fname_load)))

    % Run color tracking script
    color_tracker_4colors_script;

    % Save
    markersFilename = [meta.monkey '_' meta.date '_markers_' meta.taskAlias{fileIdx}];
    fname_save=fullfile(meta.folder,'ColorTracking','Markers',[markersFilename '.mat']);
    save(fname_save,'all_medians','all_medians2','led_vals','times');

    if first_time
        fname_save_settings=fullfile(meta.folder,'ColorTracking','Markers',['settings_' meta.monkey '_' meta.date]);
        save(fname_save_settings,'red_elbow_dist_from_blue','red_blue_arm_dist_max',...
            'green_hand_dists_elbow','red_hand_dists_elbow','blue_hand_dists_elbow','yellow_hand_dists_elbow','green_separator',...
            'green_hand_dists_bluearm','red_hand_dists_bluearm','blue_hand_dists_bluearm','yellow_hand_dists_bluearm',...
            'green_hand_dists_redarm', 'red_hand_dists_redarm', 'blue_hand_dists_redarm','yellow_hand_dists_redarm',...
            'green_dist_min','red_keep','green_keep','blue_keep','yellow_keep','marker_inits');
        clearvars -except meta altMeta 'red_elbow_dist_from_blue' 'red_blue_arm_dist_max'...
            'green_hand_dists_elbow' 'red_hand_dists_elbow' 'blue_hand_dists_elbow' 'yellow_hand_dists_elbow' 'green_separator' ...
            'green_hand_dists_bluearm' 'red_hand_dists_bluearm' 'blue_hand_dists_bluearm' 'yellow_hand_dists_bluearm' ...
            'green_hand_dists_redarm'  'red_hand_dists_redarm'  'blue_hand_dists_redarm' 'yellow_hand_dists_redarm' ...
            'green_dist_min' 'red_keep' 'green_keep' 'blue_keep' 'yellow_keep' 'marker_inits'
        first_time = 0;
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars -except meta altMeta

%% Split files and move to Final folder before loading
processSpikesForSorting(fullfile(meta.folder,'preCDS','merging'),meta.neuralPrefix);
if exist('altMeta','var') && ~isempty(altMeta.array)
    processSpikesForSorting(fullfile(altMeta.folder,'preCDS','merging'),altMeta.neuralPrefix);
end

% copy into final folder
for fileIdx = 1:length(meta.taskAlias)
    movefile(fullfile(meta.folder,'preCDS','merging',[meta.neuralPrefix '_' meta.taskAlias{fileIdx} '.mat']),...
        fullfile(meta.folder,'preCDS','Final'));
    if exist('altMeta','var') && ~isempty(altMeta.array)
        movefile(fullfile(altMeta.folder,'preCDS','merging',[altMeta.neuralPrefix '_' altMeta.taskAlias{fileIdx} '.mat']),...
            fullfile(altMeta.folder,'preCDS','Final'));
    end
    movefile(fullfile(meta.folder,'preCDS','merging',[meta.neuralPrefix '_' meta.taskAlias{fileIdx} '.nev']),...
        fullfile(meta.folder,'preCDS','Final'));
    if exist('altMeta','var') && ~isempty(altMeta.array)
        movefile(fullfile(altMeta.folder,'preCDS','merging',[altMeta.neuralPrefix '_' altMeta.taskAlias{fileIdx} '.nev']),...
            fullfile(altMeta.folder,'preCDS','Final'));
    end
end

%% Load data into CDS file
% Make CDS files
cds = cell(size(meta.taskAlias));
for fileIdx = 1:length(meta.taskAlias)
    cds{fileIdx} = commonDataStructure();
    cds{fileIdx}.file2cds(fullfile(meta.folder,'preCDS','Final',[meta.neuralPrefix '_' meta.taskAlias{fileIdx}]),...
        ['ranBy' meta.ranBy],['array' meta.array],['monkey' meta.monkey],meta.lab,'ignoreJumps',['task' meta.task],...
        ['mapFile' meta.mapfile]);

    % also load second file if necessary
    if exist('altMeta','var')
        if ~isempty(altMeta.array)
            cds{fileIdx}.file2cds(fullfile(altMeta.folder,'preCDS','Final',[altMeta.neuralPrefix '_' altMeta.taskAlias{fileIdx}]),...
                ['ranBy' altMeta.ranBy],['array' altMeta.array],['monkey' altMeta.monkey],altMeta.lab,'ignoreJumps',['task' altMeta.task],['mapFile' altMeta.mapfile]);
        else
            cds{fileIdx}.file2cds(fullfile(altMeta.folder,'preCDS','Final',[altMeta.neuralPrefix '_' altMeta.taskAlias{fileIdx}]),...
                ['ranBy' altMeta.ranBy],['monkey' altMeta.monkey],altMeta.lab,'ignoreJumps',['task' altMeta.task],['mapFile' altMeta.mapfile]);
        end
    end
end

%% Load marker file and create TRC
for fileIdx = 1:length(meta.taskAlias)
    markersFilename = [meta.monkey '_' meta.date '_markers_' meta.taskAlias{fileIdx} '.mat'];
    affine_xform = cds{fileIdx}.loadRawMarkerData(fullfile(meta.folder,'ColorTracking','Markers',markersFilename));
    writeTRCfromCDS(cds{fileIdx},fullfile(meta.folder,'OpenSim',[meta.monkey '_' meta.date '_' meta.taskAlias{fileIdx} '_markerData.trc']))
    writeHandleForceFromCDS(cds{fileIdx},fullfile(meta.folder,'OpenSim',[meta.monkey '_' meta.date '_' meta.taskAlias{fileIdx} '_handleForce.mot']))
end

%% Do openSim stuff and save analysis results to analysis folder

% do this in opensim for now

%% Add kinematic information to CDS
for fileIdx = 1:length(meta.taskAlias)
    % load joint information
    cds{fileIdx}.loadOpenSimData(fullfile(meta.folder,'OpenSim','Analysis'),'joint_ang')

    % load joint velocities
    cds{fileIdx}.loadOpenSimData(fullfile(meta.folder,'OpenSim','Analysis'),'joint_vel')

    % load joint moments
    % cds{fileIdx}.loadOpenSimData(fullfile(meta.folder,'OpenSim','Analysis'),'joint_dyn')

    % load muscle information
    cds{fileIdx}.loadOpenSimData(fullfile(meta.folder,'OpenSim','Analysis'),'muscle_len')

    % load muscle velocities
    cds{fileIdx}.loadOpenSimData(fullfile(meta.folder,'OpenSim','Analysis'),'muscle_vel')
    
%     % load hand positions
%     cds{fileIdx}.loadOpenSimData(fullfile(meta.folder,'OpenSim','Analysis'),'hand_pos')
%     
%     % load hand velocities
%     cds{fileIdx}.loadOpenSimData(fullfile(meta.folder,'OpenSim','Analysis'),'hand_vel')
%     
%     % load hand accelerations
%     cds{fileIdx}.loadOpenSimData(fullfile(meta.folder,'OpenSim','Analysis'),'hand_acc')
end

%% Save CDS
save(fullfile(meta.folder,'CDS',[meta.neuralPrefix '_CDS.mat']),'cds','-v7.3')

%% Make TD
% Bumpcurl
% params.array_alias = {'LeftS1Area2','S1'};
% params.event_list = {'ctrHoldBump';'bumpTime';'bumpDir';'ctrHold'};
% td_meta = struct('task',meta.task,'epoch','BL');
% params.trial_results = {'R','A','F','I'};
% params.meta = td_meta;
% trial_data_BL = parseFileByTrial(cds{1},params);
% params.meta.epoch = 'AD';
% trial_data_AD = parseFileByTrial(cds{2},params);
% params.meta.epoch = 'WO';
% trial_data_WO = parseFileByTrial(cds{3},params);
% 
% trial_data = cat(2,trial_data_BL,trial_data_AD,trial_data_WO);

% RT3D
% params.array_alias = {'LeftS1Area2','S1'};
% params.trial_results = {'R','A','F','I'};
% td_meta = struct('task',meta.task);
% params.meta = td_meta;
% params.bin_size = 0.01;
% params.include_ts = true;
% 
% idx_R_config = 303;
% idx_R = find(contains(cds{1}.trials.result,'R'));
% ixd_trial_config = 298; %cds{1}.trials.number(idx_R(idx_R_config));
% params.task_config = [ones(ixd_trial_config,1); 2*ones(size(cds{1}.trials,1)-ixd_trial_config,1)];
% % 1 vertical, 2 horizontal
% trial_data = parseFileByTrial(cds{1},params);

% COC3D
params.array_alias = {'LeftS1Area2','S1'};
params.trial_results = {'R','A','F','I'};
params.bin_size = 0.01;
params.include_ts = true;
params.event_list = {'stOn','stHold','goCue','stLeave','otHold','goBackCue','otLeave','ftHold','IMUreset'}';
td_meta = struct('task',meta.task);
params.meta = td_meta;

params.meta.epoch = '2D';
trial_data_2D = parseFileByTrial(cds{1},params);
params.meta.epoch = '3D';
trial_data_3D = parseFileByTrial(cds{2},params);
trial_data = [trial_data_2D trial_data_3D];

%% Save TD
save(fullfile(meta.folder,'TD',[meta.monkey '_' meta.date '_TD.mat']),'trial_data')

%% Test cds
cds{1} = commonDataStructure();
cds{1}.file2cds('C:\Users\vct1641\Documents\Data\data-tests\20180223_DBTest_COC3D_003.nev',['array' 'S1'],6,['task','COC3D']);


