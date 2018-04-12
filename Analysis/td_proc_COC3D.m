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
if strcmp(computer, 'MACI64')
    load('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/Han_20180228_TD.mat')
    trial_data = trial_datam;
    addpath('/Users/virginia/Documents/MATLAB/LIMBLAB/Data/Figs/')
else
    load(path);
end

%% Parameters
clear params

params = struct( ...
    'array',        'S1',...
    'out_signals',  'S1_spikes',...
    'space',        '3D',...
    'targ_dir',     [-pi/4, 0, pi/4, pi/2, 3*pi/4, pi, -3*pi/4],...
    'targ_dir_all', [-3*pi/4, -pi/2, -pi/4, 0, pi/4, pi/2, 3*pi/4, pi],...
    'xBound',       [-0.1 2],...
    'num_bins',     8,...
    'trim_win',     [0, 1],...
    'bin_size',     trial_data(1).bin_size);

%% Unit Raster
neu_2802_001 = [1,2,5,6,8,9,10,11,12,13,16,17,20,22,25];
neu_2802_002 = [1,5,6,8,9,10,11,13,14,16,17,20,21,22,25];
neu_2802 = [5,8,9,10,16,17,20,25];

%neuronsamp = [1:size(trial_data(1).S1_spikes,2)];
neuronsamp = [5,16,17,25];

epochs = {'2D','3D'};
movems = {'CO','OC'};
params.idx_raster = {'stLeave','otHold','otLeave','ftHold'};
params.idx_raster_col = {'g','m','c','y'};
params.idx_raster_bound = {'stLeave','otHold'};

savefig = 1;

for i = 1:length(epochs)
    
    params.epoch = epochs{i};
    
    for j = 1:length(movems)
        if strcmp(movems{j},'CO')
            params.align = 'stLeave';
            params.sortt = 'otHold';
            params.xBound = [-0.1,1.5];
            params.idx_raster = {'stLeave','otHold','otLeave','ftHold'};
            params.idx_raster_col = {'g','r','b','m'};
        else
            params.align = 'otLeave';
            params.sortt = 'ftHold';
            params.xBound = [-0.1,1];
            params.idx_raster = {'otLeave','ftHold'};
            params.idx_raster_col = {'b','m'};
        end
        
        td = removeBadTrials(trial_data);
        params.movem = movems{j};
        td = removeSpontTrials(td,params);
        
        for k = 1:length(neuronsamp)
            params.neuron = neuronsamp(k);
            [~,td] = getTDidx(td,'epoch',params.epoch);
            unitRaster(td,params);
            %epochRaster(td,params);
            if savefig
                figname = [meta.monkey,'_',meta.date,'_n',num2str(neuronsamp(k)),'_',params.epoch,'_',params.movem,'_Raster.png'];
                if strcmp(computer,'MACI64')
                    saveas(gcf,['/Users/virginia/Documents/MATLAB/LIMBLAB/Data/Figs/',figname])
                else
                    saveas(gcf,['C:\Users\vct1641\Documents\Figs\',figname])
                end
            end
        end
    end 
end

%% Tuning curves
epochs = {'2D','3D'};
movems = {'CO','OC'};
params.trim_win = [0, 0];
j = 0;

for k = 1:length(movems)
    for i = 1:length(epochs)
        params.epoch = epochs{i};
        
        td = removeBadTrials(trial_data);
        params.movem = movems{k};
        td = removeSpontTrials(td,params);
        
        [~,td] = getTDidx(td,'epoch',params.epoch);
        
        if strcmp(params.movem,'CO')
            td = trimTD(td,{'idx_stLeave',params.trim_win(1)/params.bin_size},{'idx_otHold',params.trim_win(2)/params.bin_size});
        else
            td = trimTD(td,{'idx_otLeave',params.trim_win(1)/params.bin_size},{'idx_ftHold',params.trim_win(2)/params.bin_size});
        end
        
        j = j+1;
        tuning(j).epoch = epochs{i};
        tuning(j).movem = params.movem;
        tuning(j).ntrials = size(td,2);
        
        tunResp = getTuning3D(td,params);
        tuning(j).mean_spikes = tunResp.mean_spikes;
        tuning(j).std_err = tunResp.std_err;
        tuning(j).bins = tunResp.bins;
        tuning(j).PD = tunResp.PD;
    end
end

%% Cos fit
savefig = 1;
neuronsamp = [5,16,17,25];
targ_dir = params.targ_dir;
targ_dir_all = params.targ_dir_all;

for k = 1:length(movems)
    
    tunmov = tuning(strcmp({tuning.movem},movems{k}));
    
    for i = 1:length(neuronsamp)
        figure
        cols = {'k','r'};
        
        for j = 1:size(tunmov,2)
            
            epoch = tunmov(j).epoch;
            movem = tunmov(j).movem;
            spikes_neu = tunmov(j).mean_spikes(neuronsamp(i),:);
            std_err_neu = tunmov(j).std_err(neuronsamp(i),:);
            bins = tunmov(j).bins;
            
            spikes_neu_dir = spikes_neu(ismember(bins,targ_dir))';
            std_err_neu_dir = std_err_neu(ismember(bins,targ_dir));
            bins_dir = rad2deg(bins(ismember(bins,targ_dir)));
            
            x0 = [1,1,1];
            tun_fun =  @(x) (x(1)+x(2)*cosd(bins_dir-x(3)))-spikes_neu_dir;
            [x,resnorm,residual,exitflag,output] = lsqnonlin(tun_fun,x0,[],[],optimset('Display','off'));
            
            bins_lin = rad2deg(linspace(min(targ_dir),max(targ_dir),1000));
            fun = @(x) (x(1)+x(2)*cosd(bins_lin-x(3)));
            
            h(j) = plot(bins_lin,fun(x),cols{j},'linewidth',1.5); hold on;
            plot(bins_dir,spikes_neu_dir,[cols{j},'*'])
            errorbar(bins_dir,spikes_neu_dir,std_err_neu_dir,[cols{j},'.'])
            xlabel('Target Direction [deg]','fontsize',12);
            ylabel('Mean Firing Rate [Hz]','fontsize',12);
            title(['Tuning: Neuron ',num2str(neuronsamp(i)),', ',movem],'fontsize',14); 
        end
        legend(h,epochs);
        ax = gca;
        ax.XTick = rad2deg(targ_dir_all);
        
        if savefig
            figname = [meta.monkey,'_',meta.date,'_n',num2str(neuronsamp(i)),'_',movem,'_Tuning.png'];
            if strcmp(computer,'MACI64')
                saveas(gcf,['/Users/virginia/Documents/MATLAB/LIMBLAB/Data/Figs/',figname])
            else
                saveas(gcf,['C:\Users\vct1641\Documents\Figs\',figname])
            end
        end
    end
end

%% Mean firing rate
savefig = 1;
targ_dir = sort(params.targ_dir);
params.do_stretch = true;
params.num_samp = 60;

for k = 1:length(movems)
    
    td = removeBadTrials(trial_data);
    params.movem = movems{k};
    td = removeSpontTrials(td,params);
    
    if strcmp(params.movem,'CO')
        tdt = trimTD(td,{'idx_stLeave',0},{'idx_stLeave',60});
    else
        tdt = trimTD(td,{'idx_otLeave',0},{'idx_otLeave',60});
    end
    
    for i = 1:length(neuronsamp)
        figure
        cols = {'k','r'};
        
        for j = 1:length(epochs)
            params.epoch = epochs{j};
            [~,td] = getTDidx(tdt,'epoch',params.epoch);
            [avg_data,~] = trialAverage(td,{'target_direction'},params);
            
            PDs = tuning(strcmp({tuning.movem},movems{k})&strcmp({tuning.epoch},epochs{j})).PD;
            avg_data_dir = avg_data(targ_dir == PDs(neuronsamp(i)));
            avg_spikes = movmean(avg_data_dir.S1_spikes(:,neuronsamp(i))/(td(1).bin_size),4);
            time_spikes = linspace(0,length(avg_spikes),length(avg_spikes))*(td(1).bin_size);
            
            h(j) = plot(time_spikes,avg_spikes,cols{j}); hold on;
            xlabel('Time [s]','fontsize',12); 
            ylabel('Mean Firing Rate [Hz]','fontsize',12);
            title(['FR: Neuron ',num2str(neuronsamp(i)),', ',params.movem],'Fontsize',14)
            xlim([min(time_spikes),max(time_spikes)])
        end
        legend(h,epochs);
        
        if savefig
            figname = [meta.monkey,'_',meta.date,'_n',num2str(neuronsamp(i)),'_',params.movem,'_FR.png'];
            if strcmp(computer,'MACI64')
                saveas(gcf,['/Users/virginia/Documents/MATLAB/LIMBLAB/Data/Figs/',figname])
            else
                saveas(gcf,['C:\Users\vct1641\Documents\Figs\',figname])
            end
        end
    end
end