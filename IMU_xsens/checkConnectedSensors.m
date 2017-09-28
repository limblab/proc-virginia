    function [devicesUsed, devIdUsed, nDevs] = checkConnectedSensors(devIdAll,children,h,isStation,isDongle,portS,device)
        childUsed = false(size(children));
        if isempty(children)
            fprintf('\n No devices found \n')
            stopAll(h,device,isStation,isDongle,portS)
            error('MTw:example:devicdes','No devices found')
        else
            % check which sensors are connected
            for ic=1:length(children)
                if h.XsDevice_connectivityState(children{ic}) == h.XsConnectivityState_XCS_Wireless
                    childUsed(ic) = true;
                end
            end
            % show wich sensors are connected
            fprintf('\n Devices rejected:\n')
            rejects = devIdAll(~childUsed);
            I=0;
            for i=1:length(rejects)
                I = find(strcmp(devIdAll, rejects{i}));
                fprintf(' %d - %s\n', I,rejects{i})
            end
            fprintf('\n Devices accepted:\n')
            accepted = devIdAll(childUsed);
            for i=1:length(accepted)
                I = find(strcmp(devIdAll, accepted{i}));
                fprintf(' %d - %s\n', I,accepted{i})
            end
            str = input('\n Keep current status?(y/n) \n','s');
            change = [];
            if strcmp(str,'n')
                str = input('\n Type the numbers of the sensors (csv list, e.g. "1,2,3") from which status should be changed \n (if accepted than reject or the other way around):\n','s');
                change = str2double(regexp(str, ',', 'split'));
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
            % if no device is connected, give error
            if sum(childUsed) == 0
                stopAll(h,device,isStation,isDongle,portS)
                error('MTw:example:devicdes','No devices connected')
            end
            % if sensors are rejected or accepted check blinking leds again
            if ~isempty(change)
                input('\n When sensors are connected (synced leds), press enter... \n');
            end
        end
        devicesUsed = children(childUsed);
        devIdUsed = devIdAll(childUsed);
        nDevs = sum(childUsed);
    end