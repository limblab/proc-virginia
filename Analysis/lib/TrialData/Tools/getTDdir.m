function[td] = getTDdir(trial_data,params)

targ_dir = 0;
if nargin > 1, assignParams(who,params); end % Overwrite parameters
count = 1;
for i = 1:length(trial_data)
    if trial_data(i).target_direction == targ_dir
        td(count) = trial_data(i);
        count = count+1;
    end
end
if count == 1
    fprintf('\nNo trials with target direction = %1.3f',targ_dir)
    td = [];
end
end
    