%% Load data
clear all

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

filename = [meta.monkey '_' meta.date '_TD.mat'];
path = [meta.folder '\TD\' filename];

% Load trial table
load(path);

%% Parameters
clear params

params = struct( ...
    'array',        'S1',...
    'out_signals',  'S1_spikes',...
    'space',        '3D',...
    'targ_dir',     [-pi/4, 0, pi/4, pi/2, 3*pi/4, pi, -3*pi/4],...
    'xBound',       [-1 3],...
    'num_bins',     8,...
    'trim_win',     [0, 1],...
    'bin_size',     trial_data(1).bin_size);

%% Unit Raster
neu_2802_001 = [1,2,5,6,8,9,10,11,12,13,16,17,20,22,25];
neu_2802_002 = [1,5,6,8,9,10,11,13,14,16,17,20,21,22,25];

%neuronsamp = [1:size(trial_data(1).S1_spikes,2)];
neuronsamp = neu_2802_002;

params.align = 'stLeave';
params.idx_raster = {'stLeave','otHold','otLeave','ftHold'};
params.idx_raster_col = {'g','m','c','y'};
params.idx_raster_bound = {'stLeave','otHold'};
params.epoch = '3D';

td = removeBadTrials(trial_data);

for j = 1:length(neuronsamp)
    params.neuron = neuronsamp(j);
    [idx,td] = getTDidx(td,'epoch',params.epoch);
    unitRaster(td,params);
    %epochRaster(td,params);
end

%% Window for tuning curves
targ_dir = params.targ_dir;
params.epoch = '2D';
params.trim_win = [0, 0];

td = removeBadTrials(trial_data);
[~,td] = getTDidx(td,'epoch',params.epoch);

td = trimTD(td,{'idx_stLeave',params.trim_win(1)/params.bin_size},{'idx_otHold',params.trim_win(2)/params.bin_size});

for idir = 1:length(targ_dir)
    [~,td_temp] = getTDidx(td,'target_direction',targ_dir(idir));
    spikes = cat(1,td_temp.S1_spikes)/td(1).bin_size;
    mean_spikes(idir,:) = mean(spikes,1);
    std_err_spikes(idir,:) = std(spikes)/sqrt(size(spikes,1));
end

%% Get cos fit
options = optimset('lsqnonlin');
targ_dir = params.targ_dir;
targ_dir_all = [-3*pi/4, -pi/2, -pi/4, 0, pi/4, pi/2, 3*pi/4, pi];

for i = 1:length(neuronsamp)    
    
    fb = mean_spikes(:,neuronsamp(i));  
    stderr = std_err_spikes(:,neuronsamp(i));
    bins = targ_dir;

    fb_dir = fb(ismember(bins,targ_dir))';
    bins_dir = rad2deg(bins(ismember(bins,targ_dir)));
    stderr_dir = stderr(ismember(bins,targ_dir));
    bins_lin = rad2deg(linspace(min(targ_dir),max(targ_dir),1000));
    
    x0 = [1,1,1];
    tun_fun =  @(x) (x(1)+x(2)*cosd(bins_dir-x(3)))-fb_dir;
    [x,resnorm,residual,exitflag,output] = lsqnonlin(tun_fun,x0,[],[],options);
    
    fun = @(x) (x(1)+x(2)*cosd(bins_lin-x(3)));
    
    figure
    plot(bins_lin,fun(x),'k--','linewidth',1.5); hold on;
    plot(bins_dir,fb_dir,'*')
    errorbar(bins_dir,fb_dir,stderr_dir,'k.')
    xticks(rad2deg(targ_dir_all))
    xlabel('Target Direction [deg]','fontsize',10);
    ylabel('Mean Firing Rate [Hz]','fontsize',10);
    title(['Tuning: Neuron ',num2str(neuronsamp(i))])
%     figname = [meta.date,'_n',num2str(neuronsamp(i)),'_c',num2str(params.task_config),'_Tuning.png'];
%     saveas(gcf,['C:\Users\vct1641\Documents\Figs\',figname])
end

%% Trial Average
%neuronsamp = [6,8,39,49];
PD_neu = [0];

targ_dir = sort(params.targ_dir);
params.do_stretch = false;
params.num_samp = 500;
win = [0,70];
%win = [-60,0];
td = removeBadTrials(trial_data);
%td = trimTD(td,{'idx_OTHoldTime',win(1)},{'idx_OTHoldTime',win(2)});
td = trimTD(td,{'idx_goCueTime',win(1)},{'idx_goCueTime',win(2)});

[idx,td] = getTDidx(td,'task_config',2);

idxrem = [];
for i = 1:length(td)
    if size(td(i).S1_spikes,1) < abs(win(1)-win(2))
        idxrem = [idxrem i];
    end
end
td(idxrem) = [];

[avg_data, cond_idx] = trialAverage(td,{'target_direction'},params);

for i = 1:length(neuronsamp)
    neuron = neuronsamp(i);
    avg_data_dir = avg_data((targ_dir==PD_neu(i)));
    avg_spikes = avg_data_dir.S1_spikes(:,neuron)/(td(1).bin_size);%
    time_spikes = linspace(win(1),win(2),length(avg_spikes))*0.01;
    
    figure
    plot(time_spikes,avg_spikes,'k')
    xlabel('Time [s]','fontsize',10); ylabel('Mean Firing Rate [Hz]','fontsize',10);
    title(['FR: Neuron ',num2str(neuronsamp(i))])
    xlim([min(time_spikes),max(time_spikes)])
%     figname = [meta.date,'_n',num2str(neuronsamp(i)),'_c',num2str(params.task_config),'_FR.png'];
%     saveas(gcf,['C:\Users\vct1641\Documents\Figs\',figname])
end

%%
params.task_config = 1;
params.trim_win = [0, 0.8];
td = removeBadTrials(trial_data);

[~,td] = getTDidx(td,'task_config',params.task_config);
[~,td] = getTDidx(td,'target_direction',0);

td = trimTD(td,{'idx_goCueTime',params.trim_win(1)/params.bin_size},{'idx_goCueTime',params.trim_win(2)/params.bin_size});
estim = zeros(1,55);
for i = 1:length(td)
    estim = estim + sum(td(i).S1_spikes,1);
end

%% Firing rate in time
neuronsamp = [6,39];
params.window = [];
params.trimmed = true;
td = removeBadTrials(trial_data);
td = trimTD(td,{'idx_goCueTime',0},{'idx_endTime',0});
PD_neu = [0,pi/4];

for i = 1:length(neuronsamp)
    params.neuron = neuronsamp(i);
    [idx,tdd] = getTDidx(td,'target_direction',PD_neu(i));
    FR = getFiringRates(tdd,params);
    fr = (FR.rate);
    time = linspace(0,length(fr),length(fr))*0.01;
    
    figure
    plot(time,fr,'k')
    xlabel('Time [s]','fontsize',10); ylabel('Firing Rate','fontsize',10);
    title(['Mean FR: Neuron ',num2str(neuronsamp(i))])
end