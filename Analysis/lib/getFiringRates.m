%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [FRtd] = getFiringRates(trial_data,params)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% will compute tuning curves using desired method
% A WORK IN PROGRESS
%
% INPUTS:
%   trial_data   : the struct
%   params       : parameter struct
%     .win       : window {'idx_OF_START',BINS_AFTER; 'idx_OF_END', BINS_AFTER}
%
% OUTPUTS:
%   fr    : firing rates for each trial (rows) and neuron (cols)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEFAULT PARAMETERS
array      =  'S1';
neuron     = 10;
bin_size = 0.01;
trimmed = false;
assignParams(who,params); % overwrite parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~trimmed
for trial = 1:length(trial_data)
    end_bin(trial) = size(trial_data(trial).([array '_spikes']),1);
end
[~,trial_mxend_bin] = max(end_bin);    
idx = 1:size(trial_data(trial_mxend_bin).([array '_spikes']),1);

% build firing rate matrix for the specified window
fr = zeros(length(trial_data),length(idx));
for trial = 1:length(trial_data)
    temp = trial_data(trial).([array '_spikes']);
    fr(trial,:) = [temp(:,neuron); zeros(length(idx)-size(trial_data(trial).([array '_spikes']),1),1)];
    %trial_data(trial).fr = fr(trial,:);
end
else
    fr = zeros(length(trial_data),size(trial_data(1).([array '_spikes']),1));
    for trial = 1:length(trial_data)
        temp = trial_data(trial).([array '_spikes']);
        fr(trial,:) = temp(:,neuron);
        %trial_data(trial).fr = fr(trial,:);
    end
end
FRtd = struct();
FRtd.rate  = mean(fr,1);
FRtd.stderr = std(fr,1)/sqrt(length(trial_data));
end