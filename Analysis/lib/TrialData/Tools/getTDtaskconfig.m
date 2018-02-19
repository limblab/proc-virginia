function[td] = getTDtaskconfig(trial_data,params)

task_config = 2;
if nargin > 1, assignParams(who,params); end % Overwrite parameters
count = 1;
for i = 1:length(trial_data)
    if trial_data(i).task_config == task_config
        td(count) = trial_data(i);
        count = count+1;
    end
end
end