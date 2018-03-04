function[] = epochRaster(trial_data, params)

array ='S1';
align = 'goCue';
neuron = 10;
targ_dir = 0;
idx_raster_bound = {};
idx_raster_col = {};

if nargin > 1, assignParams(who,params);
    if isfield(params,'targ_dir')
        targ_dir = params.targ_dir;
    end
end

figure
for j = 1:length(targ_dir)
    
    [~,td] = getTDidx(trial_data,'target_direction',targ_dir(j));
    td = trimTD(td,{['idx_',idx_raster_bound{1}],0},{['idx_',idx_raster_bound{2}],0});
    
    subplot(length(targ_dir),1,j)
    
    if ~isempty(td)
        
        numTrials = length(td);
        yMax = length(td);
        count = 0;
        
        for trialNum = 1:length(td)
            count = count + 1;
            trial = td((trialNum));
            for i = 1:length(idx_raster_bound)
                idx_Time(i) = (trial.bin_size*trial.(['idx_',idx_raster_bound{i}]));
            end
            spikes = trial.([array,'_ts']){neuron};
            for spike = 1:length(spikes)
                plot([spikes(spike), spikes(spike)], [count*yMax/numTrials, (count+.8)*yMax/numTrials], 'k');
                hold on
            end
            for i = 1:length(idx_raster_bound)
                plot([idx_Time(i), idx_Time(i)], [count*yMax/numTrials, (count+.8)*yMax/numTrials], idx_raster_col{i}, 'LineWidth', 2)
            end
        end
        xlim([-0.1 2]); ylim([0, yMax]);
        ylabel(num2str(rad2deg(targ_dir(j))),'fontsize',10);
        set(gca,'ytick',[],'fontsize',8)
        if j == 1
            title(['Raster: Neuron ',num2str(params.neuron)]);
        end
        if j == length(targ_dir)
            xlabel('Time [s]','fontsize',10);
        else
            set(gca,'xtick',[]);
        end
    end
end
end