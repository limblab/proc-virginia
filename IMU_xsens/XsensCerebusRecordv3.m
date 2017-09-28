%% function xsensCerebusRecord()
% syncs the recording from the cerebus and the xsens so that we can look at
% antenna switching issues due to rotation in terms of the antenna or
% switching between antennas

% without cbmex, functions aside

function XsensCerebusRecordv3()
    

%% cbmex intitialization
% open cbmex, load ccf, prep everything for recording

% cbmex('open');
% cbmex('trialconfig',0) % turn off the data buffer
% FN = 'E:\Data-lab1\TestData\Wireless Transmitter\20170905_Noise_tracking\20170905_RotAntenna_1M.nev'; % cerebus file name -- we'll use this as the base for the xsens
% % ccf_old = 'E:\Data-lab1\TestData\Wireless Transmitter\20170903_Noise_tracking\20170903_temporary.ccf';
% % cbmex('ccf','save',ccf_old);
% % cbmex('ccf','send','E:\Data-lab1\TestData\Wireless Transmitter\20170810 IMU noise tracking\20170810_SpikeAndContinuous.ccf');
% 
% 
% cbmex('fileconfig',FN,'',1);
% xsenslog = fopen('E:\Data-lab1\TestData\Wireless Transmitter\20170905_Noise_tracking\20170905_RotAntenna_1M.txt','wt');
% fprintf(xsenslog,'CerebusTime\t Roll\t Pitch\t Yaw\n');

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
        fprintf('\n Example dongle or station\n')
        dev = find(isDongle|isStation);
        isMtw = false; % if a station or a dongle is connected give priority to it.
    elseif any(isMtw)
        fprintf('\n Example MTw\n')
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
    h.XsDevice_enableRadio(device, 11);


  
    % check which devices are found
    children = h.XsDevice_children(device);

    % make sure at least one sensor is connected.
    devIdAll = cellfun(@(x) dec2hex(h.XsDevice_deviceId(x)),children,'uniformOutput',false);
    % check connected sensors, see which are accepted and which are
    % rejected.
     [devicesUsed, devIdUsed, nDevs] = checkConnectedSensors(devIdAll,children,h,isStation,isDongle,portS,device);
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
        fprintf('\n Used profile: %s(%.0f), version %.0f.\n',name,number,version)
        if any([availableProfiles{:,1}] ~= number)
            fprintf('\n Other available profiles are: \n')
            for iP=1:size(availableProfiles,1)
                fprintf(' Profile: %s(%.0f), version %.0f.\n',availableProfiles{iP,3},availableProfiles{iP,1},availableProfiles{iP,2})
            end
        end
    end

    
    input('Press ''enter'' when aligned with initial position')
    
    if output
        % create log file
        h.XsDevice_createLogFile(device,'trial.mtb');
        fprintf('\n Logfile: %s created\n',fullfile(cd,'trial.mtb'));
        
        
        % start recording
        h.XsDevice_startRecording(device);
        % register onLiveDataAvailable event
        h.registerevent({'onLiveDataAvailable',@handleData});
        h.setCallbackOption(h.XsComCallbackOptions_XSC_LivePacket, h.XsComCallbackOptions_XSC_None);
        % event handler will call stopAll when limit is reached
        input('\n Press enter to stop measurement. \n');

    else
        fprintf('\n Problems with going to measurement\n')
    end
    stopAll(h,device,isStation,isDongle,portS);

%% Event handler
    function handleData(varargin)
        % callback function for event: onLiveDataAvailable
        dataPacket = varargin{3}{2};
        deviceFound = varargin{3}{1};

        iDev = find(cellfun(@(x) x==deviceFound, devicesUsed));
        if dataPacket
            if h.XsDataPacket_containsOrientation(dataPacket)
                oriC = cell2mat(h.XsDataPacket_orientationEuler_1(dataPacket));
                %fprintf(xsenslog,'%f\t %f\t %f\t %f\n',cbmex('time'),oriC(1),oriC(2),oriC(3));
                fprintf('%f\t %f\t %f\n',oriC(1),oriC(2),oriC(3));
            end

            h.liveDataPacketHandled(deviceFound, dataPacket);


        end
    end
end