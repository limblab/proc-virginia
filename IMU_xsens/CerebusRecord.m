
reset = 0;

cbmex('open');
cbmex('trialconfig',0) % Turn off the data buffer to cbmex ?

% cbmex('ccf', 'send', 'testIMU.ccf'); % Send .ccf to the NSP

% cbmex('mask', 0, 0);
% cbmex('mask', 152, 1);

% FN = 'C:\Users\limblab\Documents\GitHub\proc\proc-Virginia\IMU_xsens\20171115_IMUreset.nev'; % Cerebus file name
FN = 'C:\data\IMU\20171120_resettest.txt';

cbmex('fileconfig',FN,'',1); % Start file recording the specified file

cbmex('trialconfig',1) % Turn on the data buffer to cbmex

%% Inside handledata
event_data = cbmex('trialdata', 1); % Read some event data, reset time for the next trialdata/flush buffer
if ~isempty(event_data{151, 3}) % Digitalin channel = 151
reset = 1;
end

%%
cbmex('close');

%%
run_time = 30; % run for time
value = 100; % value to look for (100 = d)
channel_in = 152; % serial port = channel 152, digital = 151
channel_out = 156; % dout 1 = 153, 2 = 154, 3 = 155, 4 = 156
t_col = tic; % collection time
cbmex('open'); % open library
cbmex('trialconfig',1); % start library collecting data
start = tic();
while (run_time > toc(t_col))
pause(0.05); % check every 50ms
t_test = toc(t_col);
spike_data = cbmex('trialdata', 1); % read data
found = (value == spike_data{channel_in, 3});
if (0 ~= sum(found))
cbmex('digitalout', channel_out, 1);
pause(0.01);
cbmex('digitalout', channel_out, 0);
end
end
% close the app
cbmex('close');