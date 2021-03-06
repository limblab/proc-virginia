function binData(ex,varargin)
    %binData is a method of the experiment class, and should be found
    %in the @experiment folder with the main class definition. bin data
    %produces a binnedData object and inserts it in the experiment.bin
    %field of the experiment.
    
    %% handle variable inputs
    for i=1:2:length(varargin)
        switch varargin{i}
            case 'recalcFiringRate'
                recalcFR=true;
            otherwise
                if ischar(varargin{i})
                    temp=[num2str(i),': ',varargin{i}];
                elseif isnumeric(varargin{i})
                    temp=[num2str(i),': ',num2str(varargin{i})];
                else
                    temp=[num2str(i)];
                end
                error('binData:unrecognizedInput',['did not recognize input number ',temp])
        end
    end
    if ~exist('recalcFR','var')  
        %default to using existing firing rate
        recalcFR=false;
    end
    if isempty(ex.firingRate.data)
        %if we have no firing rate data to use, then we have to run
        %ex.calcFiringRate, whether we want to or not. This flag is also
        %used at the end of the function to determine whether the logging
        %data needs to include the firing rate configuration.
        recalcFR=true;
    end
    
    %% get the continuous data into a table:
    %start with our difined fields
    bins=[];
    for i=1:length(ex.binConfig.include)
        currLabel=ex.binConfig.include(i).field;
        if strcmp(currLabel,'units') || strcmp(currLabel,'analog') 
            continue
        end
        if isempty(ex.(currLabel).data)
            error('binData:missingData',['tried to bin field ',currLabel,', which is empty'])
        end
        if strcmp(currLabel,'trials')
            warning('binData:noMethodForTrials','trials are not continuous data and cannot be binned. Trials will be skipped. remove the trials entry in binConfig.include to suppress this message')
            continue
        end
        if isempty(ex.binConfig.include(i).which)
            includeNames=ex.(currLabel).data.Properties.VariableNames(2:end);
            temp=decimateData(ex.(currLabel).data{:,:},ex.binConfig.filterConfig);
        else
            includeNames=ex.binConfig.include(i).which;
            temp=zeros(numel(ex.(currLabel).data.t),numel(ex.binConfig.include(i).which)+1);
            temp(:,1)=ex.(currLabel).data.t;
            for j=1:length(ex.binConfig.include(i).which)
                temp(:,j+1)=ex.(currLabel).data.(ex.binConfig.include(i).which{j});
            end
            temp=decimateData(temp,ex.binConfig.filterConfig);
        end
        if isempty(bins)
            %if we don't have a time column yet
            bins=table(temp(:,1),'VariableNames',{'t'});
        end
        %now append the current data set to the continousBins:
        bins=mergeTables(bins,array2table(temp,'VariableNames',[{'t'},includeNames]));
    end
    %now get anything in 'analog' if necessary:
    analogIdx=find([strcmp({ex.binConfig.include.field},'analog')],1);
    if ~isempty(analogIdx)
        for i=1:numel(ex.analog)
            if ~isempty(ex.binConfig.include(analogIdx).which)
                mask=list2tableMask(ex.analog(i).data,ex.binConfig.include(analogIdx).which);
            else
                mask=true(1,size(ex.analog(i).data,2));
            end
            if ~isempty(find(mask,1))
                mask(1)=true;
                temp=decimateData(ex.analog(i).data{:,mask},ex.binConfig.filterConfig);
                if isempty(bins)
                    %if we don't have a time column yet
                    bins=table(temp(:,1),'VariableNames',{'t'});
                end
                mask(1)=false;
                bins=mergeTables(bins,array2table(temp,'VariableNames',[{'t'},ex.analog(i).data.Properties.VariableNames(mask)]));
            end
        end
    end
    %% get unit data if necessary:
    temp=[];
    unitIdx=find([strcmp({ex.binConfig.include.field},'units')],1);
    if ~isempty(unitIdx)
        if recalcFR
            if ex.binConfig.filterConfig.sampleRate>ex.firingRateConfig.sampleRate;
                error('binData:frequencyMismatch','The sample rate of the firing rates must be equal to or greater than the sample rate of the binned data. Recompute the firing rates with a higher sample rate, or set a lower binning frequency')
            end
            ex.calcFiringRate;
        end
        %get indices of columns in the FR matrix that we want:
        if ~isempty(ex.binConfig.include(unitIdx).which)
            unitCols=zeros(1,numel(ex.binConfig.include(unitIdx).which)+1);
            unitCols(1)=find(strcmp('t',ex.firingRate.data.Properties.VariableNames));
            for i=1:numel(ex.binConfig.include(unitIdx).which)
                unitNum=ex.binConfig.include(unitIdx).which(i);
                unitCols(i+1)=find(strcmp(ex.units.getUnitName(unitNum),ex.firingRate.data.Properties.VariableNames));
            end
        else
            unitCols=1:size(ex.firingRate.data,2);
        end
        
        %decimate the Fr data if necessary:
        if ex.binConfig.filterConfig.sampleRate<ex.firingRate.meta.sampleRate
            temp=decimateData(ex.firingRate.data{:,unitCols},ex.binConfig.filterConfig);
            temp=array2table(temp,'VariableNames',ex.firingRate.data.Properties.VariableNames(unitCols));
            temp.Properties.VariableUnits=ex.firingRate.data.Properties.VariableUnits(unitCols);
            temp.Properties.VariableDescriptions=ex.firingRate.data.Properties.VariableDescriptions(unitCols);
            temp.Properties.Description=ex.firingRate.data.Properties.Description;
        else
            temp=ex.firingRate.data(:,unitCols);
        end
        %now find the common time range:
        tFR=temp.t(find(sum(isnan(temp{:,:}),2),1));%first row without nans- nans will be from lags and should occur at start and end
        if isempty(tFR)
            %if there were no NaN values in the FR matrix, simply use the
            %first time point
            tFR=temp.t(1);
        end
        tCont=bins.t(1);
        tStart=max(tFR,tCont);
        tFR=temp.t(find(sum(~isnan(temp{:,:})==size(temp,2),2),1,'last'));
            %last row without nans-> sum(~isnan(temp{:,:}) will generate a col
            %vector where each element is N-m (where N is the number of columns
            %in temp and m is the number of NaN values in that row of temp) The
            %above line finds the last time that we have a complete row with no
            %NaN values by checking that the sum of ~isnan for the rows is
            %equal to the number of cols
        if isempty(tFR)
            %if there were no NaN values in the FR matrix, simply use the
            %last point
            tFR=temp.t(end);
        end
        bins.t=roundTime(bins.t,median(diff(bins.t)));
        temp.t=roundTime(temp.t,median(diff(bins.t)));
        tCont=bins.t(end);
        tEnd=min(tFR,tCont);
        contStart=find(bins.t>=tStart,1);
        contEnd=find(bins.t<=tEnd,1,'last');
        FRStart=find(temp.t>=tStart,1);
        FREnd=find(temp.t<=tEnd,1,'last');
        
        %put the joint continuous and unit bins into ex.bin.data
        ex.bin.updateBins([bins(contStart:contEnd,:),temp(FRStart:FREnd,~strcmp('t',temp.Properties.VariableNames))])
    else
        %if no units flag was evident, just put the analog data into bins
        ex.bin.updateBins(bins);
    end
    %% notify the appended event so listners can log the operation
    evntData=loggingListenerEventData('binData',ex.binConfig);
    notify(ex,'ranOperation',evntData)
end