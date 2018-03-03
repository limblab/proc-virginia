function [t_align] = alignTD(trial_data, params)

array ='S1';
align = 'goCueTime';
if nargin > 1, assignParams(who,params); end

for i = 1:length(trial_data)
    trial = trial_data(i);
    alignStart = (trial.(['idx_',align])-trial.idx_startTime);
    
   t_align = ([1:trial.idx_endTime] - alignStart)*trial.bin_size;
end
end