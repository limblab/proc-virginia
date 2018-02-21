function [h] = unitRaster(trial_data, params)

xBound = [-.8, .8];
array ='S1';
align = 'goCueTime';
neuron = 10;
tdir = 0;
barX = 0;
if nargin > 1, assignParams(who,params);
    if isfield(params,'targ_dir')
        tdir = params.targ_dir;
    end
end

h=figure;
for j = 1:length(tdir)
    
    [idx,td] = getTDidx(trial_data,'target_direction',tdir(j)); 

    subplot(length(tdir),1,j)

    if ~isempty(td)
    
    numTrials = length(td);
    yMax = length(td);
    endTimes = zeros(1,length(td));
    count=0;
    
    for trialNum = 1:length(td)
        alignStart = td(trialNum).bin_size * (td(trialNum).(['idx_',align])-td(trialNum).idx_startTime);
        endTimes(trialNum) = td(trialNum).bin_size * td(trialNum).idx_endTime - alignStart;
    end
    [~,idx_sort] = sort(endTimes,'descend');
    
    for trialNum = 1:length(td)
        count = count + 1;
        trial = td(idx_sort(trialNum));
        alignStart = trial.bin_size * (trial.(['idx_',align])-trial.idx_startTime);
        startTime = (trial.bin_size*trial.idx_startTime) - alignStart;
        endTime = (trial.bin_size*trial.idx_endTime) - alignStart;
        goCueTime = (trial.bin_size*trial.idx_goCueTime) - alignStart;
        spikes = trial.([array,'_ts']){neuron} - alignStart;
        spikes = spikes(spikes>xBound(1) & spikes<xBound(2));
        for spike = 1:length(spikes)
            plot([spikes(spike), spikes(spike)], [count*yMax/numTrials, (count+.8)*yMax/numTrials], 'k');
            hold on
        end
            plot([startTime, startTime],  [count*yMax/numTrials, (count+.8)*yMax/numTrials], 'b', 'LineWidth', 2)
            plot([endTime, endTime],  [count*yMax/numTrials, (count+.8)*yMax/numTrials], 'r', 'LineWidth', 2)
            plot([goCueTime, goCueTime],  [count*yMax/numTrials, (count+.8)*yMax/numTrials], 'g', 'LineWidth', 2)

    end
    xlim(xBound); ylim([0, yMax]);
    ylabel([num2str(rad2deg(tdir(j)))],'fontsize',10);
    set(gca,'ytick',[],'fontsize',8)
    if j == 1
        title(['Raster: Neuron ',num2str(params.neuron)]);
    end
    if j == length(tdir)
        xlabel('Time [s]','fontsize',10);
    else
        set(gca,'xtick',[]);
    end
    end
end
end
