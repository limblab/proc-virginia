    function stopAll(h,device,isStation,isDongle,portS)
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
        fprintf('\n Close log file \n');
        h.XsDevice_closeLogFile(device);
        % on close, devices go to config mode.
        fprintf('\n Close port \n');
        % close port
        h.XsControl_closePort(portS);
        % close handle
        h.XsControl_close();
        % delete handle
        delete(h);
    end