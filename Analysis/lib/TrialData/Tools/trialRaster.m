function fhRaster = trialRaster(trial,params)
%     params.figureHandle;
%     cla(gca);
    barX = 0;
    array = 'S1';
    if nargin > 1, assignParams(who,params); end % overwrite parameters
    figure
    hold on
    count = 0;
    plot([barX, barX], [0, length(trial.([array, '_spikes'])(1,:))+1], 'r', 'LineWidth', 2)
    for neuron = 1:length(trial.([array, '_spikes'])(1,:))
        count = count +1;
       for spike = 1:length(trial.([array, '_ts']){neuron})
           plot([trial.([array, '_ts']){neuron}(spike), trial.([array, '_ts']){neuron}(spike)], [count, count+.8], 'k')
       end
    end
    %xlim([-.2, trial.idx_endTime* trial.bin_size])
    ylim([0,count+1])
    fhRaster = gca;
end