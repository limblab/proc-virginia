%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [tuning,bins] = getTuningCurves3DRT(trial_data,params)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEFAULT PARAMETERS
use_trials        =  1:length(trial_data);
num_bins        = 8;
space = '2D';
array = 'S1';
task = '3DRT';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Some undocumented parameters?
if nargin > 1, assignParams(who,params); end % overwrite parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
% get bins
bins = linspace(-pi,pi,num_bins+1);
bins = bins(2:end);
bin_spacing = unique(diff(bins));
if numel(bin_spacing)>1
    error('Something went wrong...')
end
if strcmp(task,'RT3D')
    bins(bins == -pi/2)= [];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dir = [];
    for i = 1:size(trial_data,2)
        trial_bins(i) = size(trial_data(i).([array,'_spikes']),1);
        dir = [dir; trial_data(i).target_direction*ones(trial_bins(i),1)];
    end

time_seg = mean(trial_bins)*0.01;
% bin directions
dir_bins = round(dir/bin_spacing)*bin_spacing;
dir_bins(dir_bins==-pi) = pi;
neu_num = size(trial_data(1).([array '_spikes']),2);

% find response_var in each bin, along with CI
for i = 1:length(bins)
    [~,td_bin] =  getTDidx(trial_data,'target_direction',bins(i)); 
    response_var_in_bin = zeros(1,neu_num);
    for j = 1:length(td_bin)
        response_var_in_bin = response_var_in_bin + sum(td_bin(j).([array '_spikes']),1); 
    end
    response_var_in_bin = response_var_in_bin';

    binnedResponse(:,i) = response_var_in_bin; % mean firing rate
    binned_stderr(:,i) = std(response_var_in_bin,0,2)/sqrt(size(response_var_in_bin,2)); % standard error
    tscore = tinv(0.975,size(response_var_in_bin,2)-1); % t-score for 95% CI
    binned_CIhigh(:,i) = binnedResponse(:,i)+tscore*binned_stderr(:,i); %high CI
    binned_CIlow(:,i) = binnedResponse(:,i)-tscore*binned_stderr(:,i); %low CI
end

% set up output struct

tuning.bins = bins;
tuning.binnedResponse = binnedResponse;
tuning.binned_stderr = binned_stderr;
tuning.binned_CIhigh = binned_CIhigh;
tuning.binned_CIlow = binned_CIlow;

end%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
