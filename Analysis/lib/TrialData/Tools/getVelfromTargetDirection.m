%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function trial_data = getTargetDirection(trial_data, params)
% 
%   Computes time-varying target direction (relative to hand position) for
% each trial in trial_data. So, it's a time-varying angle. Another function
% that is pretty Matt-specific at the moment, but it may be useful for
% others. Really only for center out, too.
%
% INPUTS:
%   trial_data : the struct
%   params     : struct of parameters
%     .reach_distance : how far the target is from the center
%     .dt             : size of bins in trial_data in s
%     .hold_time      : length of outer hold time in s
%
% OUTPUTS:
%   trial_data : struct with 'targ' field added
% 
% Written by Matt Perich. Updated Feb 2017.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function trial_data = getVelfromTargetDirection(trial_data, params)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set defaults and process parameter inputs
reach_distance  =  10; % cm
%hold_time       =  0.5; % s
%dt              =  0.01; % s
if nargin > 1, assignParams(who,params); end % overwrite defaults

for i = 1:length(trial_data)
    time_reach = (trial_data(i).idx_OTHoldTime-trial_data(i).idx_goCueTime)*trial_data(i).bin_size;
    vel = reach_distance/time_reach;
    
    theta = trial_data(i).target_direction;
    phi = trial_data(i).target_direction_inc;
    vel_vec = vel*[sin(theta)*cos(phi),sin(theta)*sin(phi),cos(phi)];
    
%     tgt_pos = repmat([reach_distance*cos(trial_data(i).target_direction), reach_distance*sin(trial_data(i).target_direction)],size(trial_data(i).pos,1),1);
%     pos = trial_data(i).pos;
%     targ = atan2(tgt_pos(:,2)-pos(:,2),tgt_pos(:,1)-pos(:,1));
% 
%     % it's undefined once he enters outer target
%     targ(trial_data(i).idx_trial_end - ceil(hold_time/dt):end) = 0;
% 
%     trial_data(i).targ = targ;
    
    trial_data(i).velTarg = vel;
    trial_data(i).velTarg_vec = vel_vec;
    
end

% restore logical order
trial_data = reorderTDfields(trial_data);
