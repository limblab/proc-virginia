function [trial_data,bad_trials] = removeSpontTrials(trial_data,params)

movem = 'CO';
if nargin > 1, assignParams(who,params); end % overwrite defaults

bad_idx = false(1,length(trial_data));
for trial = 1:length(trial_data)
    err = false;
    
    td = trial_data(trial);
    
    if strcmp(td.task,'RT3D')
        if td.idx_endTime < 100
            err = true;
        end
    end
    
    if strcmp(td.task,'COC3D')
        if strcmp(movem, 'CO')
            if (td.idx_otHold-td.idx_stLeave)*td.bin_size <= 0.21
                err = true;
            end
        elseif strcmp(movem, 'OC')
            if (td.idx_ftHold-td.idx_otLeave)*td.bin_size <= 0.21
                err = true;
            end
        end
    end
    
    
    if err, bad_idx(trial) = true; end
end
disp(['Removing ' num2str(sum(bad_idx)) ' trials.']);
bad_trials = trial_data(bad_idx);
trial_data = trial_data(~bad_idx);