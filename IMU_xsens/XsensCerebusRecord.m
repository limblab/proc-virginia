%% function xsensCerebusRecord()
% syncs the recording from the cerebus and the xsens so that we can look at
% antenna switching issues due to rotation in terms of the antenna or
% switching between antennas


function XsensCerebusRecord()
%% cbmex intitialization
% open cbmex, load ccf, prep everything for recording

cbmex('open');
reccbmex = 1;
if reccbmex
    cbmex('trialconfig',0) % turn off the data buffer
    FN = 'C:\Users\limblab\Documents\GitHub\proc\proc-Virginia\IMU_xsens\20171120_testcmbex.nev'; % cerebus file name
end

xsenslog = fopen('C:\Users\limblab\Documents\GitHub\proc\proc-Virginia\IMU_xsens\txt\20171120_testcmbex3.txt','wt'); % xsens file name
fprintf(xsenslog,'DevID\t CerebusTime\t Roll\t Pitch\t Yaw\t xAcc\t yAcc\t zAcc\t xGyro\t yGyro\t zGyro\t xMagn\t yMagn\t zMagn\t q0\t q1\t q2\t q3\n'); % xsens header

%% Launching activex server
switch computer
    case 'PCWIN'
        serverName = 'xsensdeviceapi_com32.IXsensDeviceApi';
    case 'PCWIN64'
        serverName = 'xsensdeviceapi_com64.IXsensDeviceApi';
end
h = actxserver(serverName);
fprintf( '\n ActiveXsens server - activated \n' );

version = h.XsControl_version;
fprintf(' XDA version: %.0f.%.0f.%.0f\n',version{1:3})
if length(version)>3
    fprintf(' XDA build: %.0f %s\n',version{4:5});
end

%% Scanning connection ports
% ports rescanned must be reopened
p_br = h.XsScanner_scanPorts(0, 100, true, true);
fprintf( '\n Connection ports - scanned \n' );

% check using device id's what kind of devices are connected.
isMtw = cellfun(@(x) h.XsDeviceId_isMtw(x),p_br(:,1));
isDongle = cellfun(@(x) h.XsDeviceId_isAwindaDongle(x),p_br(:,1));
isStation = cellfun(@(x) h.XsDeviceId_isAwindaStation(x),p_br(:,1));

if any(isDongle|isStation)
    fprintf('\n Found dongle or station\n')
    dev = find(isDongle|isStation);
    isMtw = false; % if a station or a dongle is connected give priority to it.
elseif any(isMtw)
    fprintf('\n Found MTw\n')
    dev = find(isMtw);
else
    fprintf('\n No device found. \n')
    h.XsControl_close();
    delete(h);
    return
end

% port scan gives back information about the device, use first device found.
deviceID = p_br{dev(1),1};
portS = p_br{dev(1),3};
baudRate = p_br{dev(1),4};

devTypeStr = '';
if any(isMtw)
    devTypeStr = 'MTw';
elseif any(isDongle)
    devTypeStr = 'dongle';
else
    assert(any(isStation))
    devTypeStr = 'station';
end
fprintf('\n Found %s on port %s, with ID: %s and baudRate: %.0f \n',devTypeStr, portS, dec2hex(deviceID), baudRate);

% open port
if ~h.XsControl_openPort(portS, baudRate, 0 ,true)
    fprintf('\n Unable to open port %s. \n', portS);
    h.XsControl_close();
    delete(h);
    return;
end

%% Initialize Master Device
% get device handle.
device = h.XsControl_device(deviceID);

% To be able to get orientation data from a MTw, the filter in the
% software needs to be turned on:
h.XsDevice_setOptions(device, h.XsOption_XSO_Orientation, 0);
h.XsDevice_gotoConfig(device);

% Set the update rate to 120 Hz
h.XsDevice_setUpdateRate(device, 120);
% Set radio to channel 11
h.XsDevice_enableRadio(device, 15);

pause(3.5);

% check which devices are found
children = h.XsDevice_children(device);

% make sure at least one sensor is connected.
devIdAll = cellfun(@(x) dec2hex(h.XsDevice_deviceId(x)),children,'uniformOutput',false);
% check connected sensors, see which are accepted and which are
% rejected.
[devicesUsed, devIdUsed, nDevs] = checkConnectedSensors(devIdAll);
fprintf(' Used device: %s \n',devIdUsed{:});


%% Entering measurement mode
fprintf('\n Activate measurement mode \n');
% goto measurement mode
output = h.XsDevice_gotoMeasurement(device);

% display radio connection information
if(any(isDongle|isStation))
    fprintf('\n Connection has been established on channel %i with an update rate of %i Hz\n', h.XsDevice_radioChannel(device), h.XsDevice_updateRate(device));
else
    assert(any(isMtw))
    fprintf('\n Connection has been established with an update rate of %i Hz\n', h.XsDevice_updateRate(device));
end


% check filter profiles
if ~isempty(devicesUsed)
    availableProfiles = h.XsDevice_availableXdaFilterProfiles(devicesUsed{1});
    usedProfile = h.XsDevice_xdaFilterProfile(devicesUsed{1});
    number = usedProfile{1};
    version = usedProfile{2};
    name = usedProfile{3};
    fprintf(' Used profile: %s(%.0f), version %.0f.\n',name,number,version)
    if any([availableProfiles{:,1}] ~= number)
        fprintf('\n Other available profiles are: \n')
        for iP=1:size(availableProfiles,1)
            fprintf(' Profile: %s(%.0f), version %.0f.\n',availableProfiles{iP,3},availableProfiles{iP,1},availableProfiles{iP,2})
        end
    end
end

pause(1);

%Start recording cerebus
if reccbmex
    cbmex('mask', 0, 0);
    cbmex('mask', 151, 1)
    cbmex('fileconfig',FN,'',1);
    cbmex('trialconfig',1,'event',10) % Turn on the data buffer to cbmex
end

input('\n Press ''enter'' when aligned with initial position')

for i = 1:length(children)
    coord_reset(i) = h.XsDevice_resetOrientation(children{i}, h.XsResetMethod_XRM_Alignment());
end

if output && all(coord_reset)
    % create log file
    % h.XsDevice_createLogFile(device,'exampleLogfile.mtb');
    % fprintf('\n Logfile: %s created\n',fullfile(cd,'exampleLogfile.mtb'));
    
    % start recording
    %cbmex('fileconfig',FN,'',1);
    
    h.XsDevice_startRecording(device);
    
    % register onLiveDataAvailable event
    resetcount1 = 0;
    resetcount2 = 0;
    sampnum = 0; 
    h.registerevent({'onLiveDataAvailable',@handleData});
    h.setCallbackOption(h.XsComCallbackOptions_XSC_LivePacket, h.XsComCallbackOptions_XSC_None);
    % event handler will call stopAll when limit is reached
    input('\n Press enter to stop measurement');
else
    fprintf('\n Problems with going to measurement\n')
end
stopAll;

%% Event handler
    function handleData(varargin)
        % callback function for event: onLiveDataAvailable
        dataPacket = varargin{3}{2};
        deviceFound = varargin{3}{1};
        
        iDev = find(cellfun(@(x) x==deviceFound, devicesUsed));
        if dataPacket
            if h.XsDataPacket_containsOrientation(dataPacket)
                oriC = cell2mat(h.XsDataPacket_orientationEuler_1(dataPacket));
                accC = cell2mat(h.XsDataPacket_calibratedAcceleration(dataPacket));
                gyroC = cell2mat(h.XsDataPacket_calibratedGyroscopeData(dataPacket));
                magnC = cell2mat(h.XsDataPacket_calibratedMagneticField(dataPacket));
                quat = cell2mat(h.XsDataPacket_orientationQuaternion_1(dataPacket));
                fprintf(xsenslog,'%d\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n',iDev,cbmex('time'),oriC,accC,gyroC,magnC,quat);
            end
            
            sampnum = sampnum+1;
            if sampnum == 50
                event_data = cbmex('trialdata', 1); % Read some event data, reset time for the next trialdata/flush buffer
                if ~isempty(event_data{151, 3})
                    for i = 1:length(children)
                        h.XsDevice_resetOrientation(children{i}, h.XsResetMethod_XRM_Alignment());
                    end
                end
                sampnum = 0;               
            end
            
            h.liveDataPacketHandled(deviceFound, dataPacket);
       
%             if length(children)==1
%                 if all(abs(oriC)<=2)
%                     resetcount1 = resetcount1+1;
%                 end
%                 if resetcount1==10
%                     h.XsDevice_resetOrientation(children{1}, h.XsResetMethod_XRM_Alignment());
%                     resetcount1 = 0;
%                 end
%             else
%                 if all(abs(oriC)<=3)&&(iDev==1)
%                     resetcount1 = resetcount1+1;
%                 elseif all(abs(oriC)<=3)&&(iDev==2)
%                     resetcount2 = resetcount2+1;
%                 end
%                 if resetcount1==10
%                     h.XsDevice_resetOrientation(children{iDev}, h.XsResetMethod_XRM_Alignment());
%                     resetcount1 = 0;
%                 elseif resetcount2==10
%                     h.XsDevice_resetOrientation(children{iDev}, h.XsResetMethod_XRM_Alignment());
%                     resetcount2 = 0;
%                 end
                %                 if (resetcount1+resetcount2==50)&&((resetcount1>20)&&(resetcount2>20))
                %                     for i = 1:length(children)
                %                         h.XsDevice_resetOrientation(children{i}, h.XsResetMethod_XRM_Alignment());
                %                     end
                %                     resetcount1 = 0;
                %                     resetcount2 = 0;
                %                 end
                
%             end
        end
    end

    function stopAll
        % close everything in the right way
        if ~isempty(h.eventlisteners)
            h.unregisterevent({'onLiveDataAvailable',@handleData});
            h.setCallbackOption(h.XsComCallbackOptions_XSC_None, h.XsComCallbackOptions_XSC_LivePacket);
        end
        % stop recording, showing data
        fprintf('\n Stop recording, go to config mode \n');
        h.XsDevice_stopRecording(device);
        h.XsDevice_gotoConfig(device);
        % disable radio for station or dongle
        if any(isStation|isDongle)
            h.XsDevice_disableRadio(device);
        end
        % close log file
        %         fprintf('\n Close log file \n');
        %         h.XsDevice_closeLogFile(device);
        % on close, devices go to config mode.
        fprintf('\n Close port \n');
        % close port
        h.XsControl_closePort(portS);
        % close handle
        h.XsControl_close();
        % delete handle
        delete(h);
        
        % my added cbmex junk
        if reccbmex
            cbmex('fileconfig',FN,'',0)
        end
        % cbmex('ccf','send',ccf_old)
        cbmex('close')
        fclose(xsenslog);
        
    end

    function [devicesUsed, devIdUsed, nDevs] = checkConnectedSensors(devIdAll)
        childUsed = false(size(children));
        if isempty(children)
            fprintf('\n No devices found \n')
            stopAll
            error('MTw:example:devicdes','No devices found')
        else
            % check which sensors are connected
            for ic=1:length(children)
                if h.XsDevice_connectivityState(children{ic}) == h.XsConnectivityState_XCS_Wireless
                    childUsed(ic) = true;
                end
            end
            % show wich sensors are connected
            rejects = devIdAll(~childUsed);
            if ~isempty(rejects)
                fprintf('\n Devices rejected:\n')
                I=0;
                for i=1:length(rejects)
                    I = find(strcmp(devIdAll, rejects{i}));
                    fprintf(' %d - %s\n', I,rejects{i})
                end
            end
            accepted = devIdAll(childUsed);
            if ~isempty(accepted)
                fprintf('\n Devices accepted:\n')
                for i=1:length(accepted)
                    I = find(strcmp(devIdAll, accepted{i}));
                    fprintf(' %d - %s\n', I,accepted{i})
                end
            end
            str = input('\n Keep current status?(y/n) ','s');
            change = [];
            if strcmp(str,'n')
                str = input('\n Type the numbers of the sensors (csv list, e.g. "1,2,3") from which status should be changed \n (if accepted than reject or the other way around):\n','s');
                change = str2double(regexp(str, ',', 'split'));
                if isempty(str)
                    stopAll;
                else
                    for iR=1:length(change)
                        if childUsed(change(iR))
                            % reject sensors
                            h.XsDevice_rejectConnection(children{change(iR)});
                            childUsed(change(iR)) = false;
                        else
                            % accept sensors
                            h.XsDevice_acceptConnection(children{change(iR)});
                            childUsed(change(iR)) = true;
                        end
                    end
                end
            end
            % if no device is connected, give error
            if sum(childUsed) == 0
                stopAll
                error('MTw:example:devicdes','No devices connected')
            end
            % if sensors are rejected or accepted check blinking leds again
            %             if ~isempty(change)
            %                 input('\n When sensors are connected (synced leds), press enter... \n');
            %             end
        end
        devicesUsed = children(childUsed);
        devIdUsed = devIdAll(childUsed);
        nDevs = sum(childUsed);
    end
end

%% Helper function to create figure for display
% function [t, dataPlot, linePlot, packetCounter] = createFigForDisplay(nDevs, deviceIds)
%
%         [dataPlot{1:nDevs}] = deal([]);
%         [linePlot{1:nDevs}] = deal([]);
%         [t{1:nDevs}] = deal([]);
%
%         %% not more than 6 devices per plot
%         nFigs = ceil(nDevs/6);
%         devPerFig = ceil(nDevs/nFigs);
%         m = ceil(sqrt(devPerFig));
%         n = ceil(devPerFig/m);
%         lDev = 0;
%         for iFig=1:nFigs
%             figure('name',['Example MTw_' num2str(iFig)])
%             iPlot = 0;
%             for iDev = lDev+1:min(iFig*devPerFig, nDevs)
%                 iPlot = iPlot+1;
%                 ax = subplot(m,n,iPlot);
%                 linePlot{iDev} = plot(ax, 0,[NaN NaN NaN]);
%                 title(['Orientation data ' deviceIds{iDev}]), xlabel('sample'), ylabel('euler (deg)')
%                 legend(ax, 'roll','pitch','yaw');
%             end
%             lDev = iDev;
%         end
%         packetCounter = zeros(nDevs,1);
%     end
