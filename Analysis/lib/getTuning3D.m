function[tuning] = getTuning3D(trial_data,params)

targ_dir = [];
if nargin > 1, assignParams(who,params);end

for i = 1:length(targ_dir)
    [~,td] = getTDidx(trial_data,'target_direction',targ_dir(i));
    spikes = cat(1,td.S1_spikes)/td(1).bin_size;
    mean_spikes(i,:) = mean(spikes,1);
    std_err_spikes(i,:) = std(spikes)/sqrt(size(spikes,1));
    bins(i) = targ_dir(i);
end

tuning.mean_spikes = mean_spikes';
tuning.std_err = std_err_spikes';
tuning.bins = bins';
[~,idmax] = max(mean_spikes',[],2);
tuning.PD = bins(idmax)';

end