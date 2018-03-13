function[] = unitRaster(trial_data, params)

xBound = [-.8, .8];
array ='S1';
align = 'goCue';
sortt = 'EndTime';
neuron = 10;
targ_dir = 0;
idx_raster = {};
idx_raster_col = {};

if nargin > 1, assignParams(who,params);
    if isfield(params,'targ_dir')
        targ_dir = params.targ_dir;
    end
end

figure
for j = 1:length(targ_dir)
    
    [~,td] = getTDidx(trial_data,'target_direction',targ_dir(j)); 

    subplot(length(targ_dir),1,j)

    if ~isempty(td)
    
    numTrials = length(td);
    yMax = length(td);
    sortTimes = zeros(1,length(td));
    count=0;
    
    for trialNum = 1:length(td)
        alignStart = td(trialNum).bin_size * (td(trialNum).(['idx_',align])-td(trialNum).idx_startTime);
        sortTimes(trialNum) = td(trialNum).bin_size * td(trialNum).(['idx_',sortt]) - alignStart;
    end
    [~,idx_sort] = sort(sortTimes,'descend');
    
    for trialNum = 1:length(td)
        count = count + 1;
        trial = td(idx_sort(trialNum));
        alignStart = trial.bin_size * (trial.(['idx_',align])-trial.idx_startTime);
        startTime = (trial.bin_size*trial.idx_startTime) - alignStart;
        endTime = (trial.bin_size*trial.idx_endTime) - alignStart;
        for i = 1:length(idx_raster)
        idx_Time(i) = (trial.bin_size*trial.(['idx_',idx_raster{i}])) - alignStart;
        end
        spikes = trial.([array,'_ts']){neuron} - alignStart;
        spikes = spikes(spikes>xBound(1) & spikes<xBound(2));
        for spike = 1:length(spikes)
            plot([spikes(spike), spikes(spike)], [count*yMax/numTrials, (count+.8)*yMax/numTrials], 'k');
            hold on
        end
            %plot([startTime, startTime],  [count*yMax/numTrials, (count+.8)*yMax/numTrials], 'b', 'LineWidth', 2)
            %plot([endTime, endTime],  [count*yMax/numTrials, (count+.8)*yMax/numTrials], 'r', 'LineWidth', 2)
           for i = 1:length(idx_raster)
            plot([idx_Time(i), idx_Time(i)],  [count*yMax/numTrials, (count+.8)*yMax/numTrials], idx_raster_col{i}, 'LineWidth', 2)
           end
    end
    xlim(xBound); ylim([0, yMax]);
    ylabel([num2str(rad2deg(targ_dir(j)))],'fontsize',10);
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
