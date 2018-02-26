function loadOpenSimData(cds,folderPath,dataType)
    %this is a method of the cds class and should be stored in the
    %@commonDataStructure folder with the other class methods.
    %
    %attempts to load Open Sim data from the source directory of the cds.
    %This uses the meta field to try and find properly named files in the
    %folder specified. The prefix of the file must match the name of the
    %source file of the cds
    %
    % Postfix can currently be one of:
    %   'joint_ang'
    %   'joint_vel'
    %   'joint_acc'
    %   'joint_dyn'
    %   'muscle_len'
    %   'muscle_vel'
    
    
    if ~strcmp(folderPath(end),filesep)
        folderPath=[folderPath,filesep];
    end
    
    %interpolate onto times aligned with the existing kinematic data. If no
    %existing kinematic data, just use a 100hz signal aligned to zero.
    if ~isempty(cds.kin)
        dt=mode(diff(cds.kin.t));
    else
        dt=.01;
    end
    
    prefix=cds.meta.rawFileName;
    if ~iscell(prefix)
        prefix={prefix};
    end
    foundFiles={};
    for i=1:numel(prefix)
        %find and strip extensions if present
        extLoc=max(strfind(prefix{i},'.'));
        if ~isempty(extLoc)
            prefix{i}=prefix{i}(1:extLoc-1);
        end
        
        % set the right file and header postfixes
        switch(dataType)
            case 'joint_ang'
                postfix = '_Kinematics_q.sto';
                header_post = '_ang';
            case 'joint_vel'
                postfix = '_Kinematics_u.sto';
                header_post = '_vel';
            case 'joint_acc'
                postfix = '_Kinematics_dudt.sto';
                header_post = '_acc';
                error('loadOpenSimData:unsupportedDataType','Joint accelerations are currently unsupported until dynamics are added to modeling')
            case 'joint_dyn'
                postfix = '_Dynamics_q.sto';
                header_post = ''; % already postfixed by 'moment'
            case 'muscle_len'
                postfix = '_MuscleAnalysis_Length.sto';
                header_post = '_len';
            case 'muscle_vel'
                % temporary until Fiber_velocity file is fixed: take
                % gradient of muscle lengths
%                 postfix = '_MuscleAnalysis_FiberVelocity.sto';
                postfix = '_MuscleAnalysis_Length.sto';
                header_post = '_muscVel';
            otherwise
                error('loadOpenSimData:invalidDataType', 'Data type must be one of {''joint_ang'', ''joint_vel'', ''joint_dyn'', ''muscle_len''}')
        end
        fileNameList = {[folderPath,prefix{i},postfix]};
%         fileNameList={[folderPath,prefix{i},'_Kinematics_q.sto'];...
%             [folderPath,prefix{i},'_MuscleAnalysis_Length.sto']};
%             [folderPath,prefix{i},'_MuscleAnalysis_Length.sto'];...
%             [folderPath,prefix{i},'_Dynamics_q.sto']};
        for j=1:numel(fileNameList)
            foundList=dir(fileNameList{j});
            if ~isempty(foundList)
                %load data from file into table 'kin':
                fid=fopen(fileNameList{j});
                %loop through the header till we find the first row of data:
                tmpLine=fgetl(fid);
                %check for correct file given dataType
                switch(dataType)
                    case 'joint_ang'
                        if ~strcmp(tmpLine,'Coordinates')
                            error('loadOpenSimData:wrongFile',['Header in analysis file ' fileNameList{j} ' is incorrect'])
                        end
                    case 'joint_vel'
                        if ~strcmp(tmpLine,'Speeds')
                            error('loadOpenSimData:wrongFile',['Header in analysis file ' fileNameList{j} ' is incorrect'])
                        end
                    case 'joint_acc'
                        if ~strcmp(tmpLine,'Accelerations')
                            error('loadOpenSimData:wrongFile',['Header in analysis file ' fileNameList{j} ' is incorrect'])
                        end
                    case 'joint_dyn'
                        if ~strcmp(tmpLine,'Inverse Dynamics Generalized Forces')
                            error('loadOpenSimData:wrongFile',['Header in analysis file ' fileNameList{j} ' is incorrect'])
                        end
                    case 'muscle_len'
                        if ~strcmp(tmpLine,'Length')
                            error('loadOpenSimData:wrongFile',['Header in analysis file ' fileNameList{j} ' is incorrect'])
                        end
                    case 'muscle_vel'
                        % temporary until Fiber_velocity file is fixed: take
                        % gradient of muscle lengths
                        if ~strcmp(tmpLine,'Length')
                            error('loadOpenSimData:wrongFile',['Header in analysis file ' fileNameList{j} ' is incorrect'])
                        end
                    otherwise
                        error('loadOpenSimData:invalidDataType', 'Data type must be one of {''joint_ang'', ''joint_vel'', ''joint_dyn'', ''muscle_len''}')
                end
                while ~strcmp(tmpLine,'endheader')
                    if ~isempty(strfind(tmpLine,'nRows'))
                        nRow=str2double(tmpLine(strfind(tmpLine,'=')+1:end));
                    elseif ~isempty(strfind(tmpLine,'nColumns'))
                        nCol=str2double(tmpLine(strfind(tmpLine,'=')+1:end));
                    elseif ~isempty(strfind(tmpLine,'inDegrees'))
                        if ~isempty(strfind(tmpLine,'yes'))
                            unitLabel='deg';
                        else
                            unitLabel='rad';
                        end
                    end
                    tmpLine=fgetl(fid);
                end
                header=strsplit(fgetl(fid));
                %convert 'time' to 't' to match cds format:
                idx=find(strcmp(header,'time'),1);
                if isempty(idx)
                    %look for a 't' column
                    idx=find(strcmp(header,'t'),1);
                    if isempty(idx)
                        error('loadOpenSimData:noTime',['could not find a time column in the file: ', kinFileName])
                    end
                else
                    %convert 'time into 't'
                    header{idx}='t';
                end
                
                % convert header to specify type of data
                other_idx = find(~strcmp(header,'t'));
                for header_ctr = 1:length(other_idx)
                    header{other_idx(header_ctr)} = [header{other_idx(header_ctr)} header_post];
                end
                
                scanned_input = fscanf(fid,repmat('%f',[1,nCol]));
                a=reshape(scanned_input,[nCol,nRow])';
                %sanity check time:
                SR=mode(diff(a(:,1)));
                if size(a,1)~=round((1+ (max(a(:,1))-min(a(:,1)))/SR))
                    warning('loadOpenSimData:badTimeSeries',['the timeseries in the detected opensim data is missing time points. expected ',num2str((1+ (max(a(:,1))-min(a(:,1)))/SR)),' points, found ',num2str(size(a,1)),' points'])
                    disp('data will be interpolated to reconstruct missing points')
                    cds.addProblem('kinect data has missing timepoints, data in the cds has been interpolated to reconstruct them')
                end
                %interpolate to desired time vector:
                desiredTime=roundTime(a(1,1):dt:a(end,1));%uniformly samples a with spacing dt, then shifts time bins to be zero aligned
                desiredTime=desiredTime(desiredTime>min(a(:,1)) & desiredTime<max(a(:,1)))';%clear out any points that fall outside the original time window due to the shift
                
                interpData = interp1(a(:,1),a(:,2:end),desiredTime);
                
                % Temporary until fiber velocity file is fixed: take
                % gradient for muscle velocity
                if strcmp(dataType,'muscle_vel')
                    for muscle_ctr = 1:size(interpData,2)
                        grad_interpData(:,muscle_ctr) = gradient(interpData(:,muscle_ctr),desiredTime);
                    end
                    interpData = grad_interpData;
                end
                
                kin=array2table([desiredTime,interpData],'VariableNames',header);
                unitsLabels=[{'s'},repmat({unitLabel},[1,nCol-1])];
                kin.Properties.VariableUnits=unitsLabels;
                %find sampling rate and look for matching rate in analog data:
                SR=round(1/mode(diff(kin.t)));
                cdsFrequencies=zeros(1,length(cds.analog));
                for k=1:length(cds.analog)
                    cdsFrequencies(k)=round(1/mode(diff(cds.analog{k}.t)));
                end
                match=find(cdsFrequencies==SR);
                %append new data into the analog cell array:
                if isempty(match)
                    %stick the data in a new cell at the end of the cds.analog
                    %cell array:
                    cds.analog{end+1}=kin;
                else
                    %append the new data to the table with the matching
                    %frequency:
                    cds.analog{match}=mergeTables(cds.analog{match},kin);
                end
                foundFiles=[foundFiles;fileNameList(j)];
            else
                warning('loadOpenSimData:fileNotFound',['The specified file: ' fileNameList{j} ' was not found. Check file name and try again.']);
            end
        end
           
    end
    
    
    cds.sanitizeTimeWindows
    logStruct=struct('folder',folderPath,'fileNames',foundFiles);
    evntData=loggingListenerEventData('loadOpenSimData',logStruct);
    notify(cds,'ranOperation',evntData)
end