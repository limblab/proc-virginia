function getRT3DTaskTable(cds,times)
    %this is a method function for the common_data_structure (cds) class, and
    %should be located in a folder '@common_data_structure' with the class
    %definition file and other method files
    %
    %cds.getRT3DTaskTable(times)
    % returns no value, instead it populates the trials field
    %of the cds assuming the task is a two-workspace random target task. Takes a single
    %input:times, which is a table with 4 columns: number, startTime,
    %endTime, and result. These times define the start and stop of trials
    %as indicated by the state words for trial start and trial end. the
    %result code will be a character 'R':reward 'A':abort 'F':fail
    %'I':incomplete.
    
    corruptDB=0;
    numTrials = length(times.number);
    wordGo = hex2dec('31');
    goCues =  cds.words.ts(bitand(hex2dec('f0'), cds.words.word) == wordGo);

    wordCTHold = hex2dec('A0');
    ctHoldTimes = cds.words.ts(bitand(hex2dec('f0'), cds.words.word) == wordCTHold);
    
    wordTargHold = hex2dec('A1');
    targMask=bitand(hex2dec('f0'),cds.words.word) == wordTargHold;
    targHoldTimes = cds.words.ts(targMask);

    %check DB version number and run appropriate parsing code
    % DB version 0 has 25 bytes before target position
    db_version=cds.databursts.db(1,2);

        % * bytes  9 to  12: float => x offset
        % * bytes 13 to 16: float => y offset
        % * bytes 17 to 20: float => target_size
        % * bytes 21 to 24: float => workspace number
        % * bytes 25 to 25+N*8: where N is the number of targets, contains 8 bytes per 
        % *      target representing two single-precision floating point numbers in 
        % *      little-endian format represnting the x and y position of the center of 
        % *      the target.
        
        if db_version==2
        %  * Version 2 (0x02)
        %  * ----------------
        %  * byte   0: uchar => number of bytes to be transmitted
        %  * byte   1: uchar => databurst version number (in this case: 0)
        %  * byte   2 to 4: uchar => task code ('3DR')
        %  * byte   5: uchar => model version major
        %  * byte   6: uchar => model version minor
        %  * bytes  9 to 12: float => number of targets (includes first target)
        %  * bytes 13 to 16: float => start target hold time
        %  * bytes 17 to 20: float => other target hold time
        %  * bytes 21 to 21+(N)*8: where N is the number of targets, contains 4 bytes per
        %  *          target, numbered 0-7. Target 0 is center target, and outer targets
        %  *          are numbered 1-7, counter-clockwise, starting from bottom right
        %  */
        
        hdrSize=21;
        numTgt = (cds.databursts.db(1)-hdrSize)/8;

        targStartList=  nan(numTrials,1);
        goCueList=      nan(numTrials,numTgt);
        numTgts=        numTgt*ones(numTrials,1);
        numAttempted=   nan(numTrials,1);
        ftHoldTimeList=     nan(numTrials,1);
        targHoldTimeList=   nan(numTrials,1);

        for trial = 1:numel(times.startTime)
            % Find databurst associated with startTime
            dbidx = find(cds.databursts.ts > times.startTime(trial) & cds.databursts.ts < times.endTime(trial));
            if length(dbidx) > 1
                warning('trt_trial_table: multiple databursts @ t = %.3f, using first:%d',times.startTime(trial),trial);
                dbidx = dbidx(1);
            elseif isempty(dbidx)
                warning('trt_trial_table: no/deleted databurst @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            end
            if (cds.databursts.db(dbidx,1)-hdrSize)/8 ~= numTgt
                %catch weird/corrupt databursts with different numbers of targets
                warning('trt_trial_table: Inconsistent number of targets @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            end

            % Target on times
            idxTargHold = find(targHoldTimes > times.startTime(trial) & targHoldTimes < times.endTime(trial),1,'first');
            %identify trials with corrupt codes that might end up with extra
            %targets
            if isempty(idxTargHold)
                targStart = NaN;
            else
                targStart = targHoldTimes(idxTargHold);
            end
            
            % Go cues
            idxGo = find(goCues > times.startTime(trial) & goCues < times.endTime(trial));

            %get the codes and times for the go cues
            goCue = nan(1,numTgt);
            if isempty(idxGo)
                tgtsAttempted = 0;
            else
                tgtsAttempted = length(idxGo);
            end
            if tgtsAttempted>1
                goCue(1:tgtsAttempted)=goCues(idxGo);
            end

            %identify trials with corrupt end codes that might end up with extra
            %targets
            if length(idxGo) > numTgt
                warning('trt_trial_table: Inconsistent number of targets @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            end
            %find target centers
            %ctr=bytes2float(cds.databursts.db(dbidx,hdrSize+1:end));
            % Offsets, target size
%             xOffset = bytes2float(cds.databursts.db(dbidx,10:13));
%             yOffset = bytes2float(cds.databursts.db(dbidx,14:17));
%             tgtSize = bytes2float(cds.databursts.db(dbidx,18:21));
%             wsnum = bytes2float(cds.databursts.db(dbidx,22:25));

            % Get hold times 
            ftHoldTime = bytes2float(cds.databursts.db(dbidx,13:16));
            targHoldTime = bytes2float(cds.databursts.db(dbidx,17:20));
            
            % Get target numbers
            tgtnum=bytes2float(cds.databursts.db(dbidx,hdrSize+1:end));

            % Build arrays
            targStartList(trial,:)=     targStart;          % time of first target onset
            goCueList(trial,:)=         goCue;              % time stamps of go_cue(s)
            numTgts(trial)=             numTgt;             % max number of targets
            numAttempted(trial,:)=      tgtsAttempted;      % ?
            ftHoldTimeList(trial)=      ftHoldTime;         % first target hold time
            targHoldTimeList(trial)=    targHoldTime;       % outer target hold time
            tgt(trial,:)=               tgtnum;             %target number
        end

        trials=table(targStartList,goCueList,numTgts,numAttempted,ftHoldTimeList,targHoldTimeList,tgt,...
                    'VariableNames',{'targetStartTime','goCueTime','numTgt','numAttempted','ftHoldTime','targHoldTime','tgtnum'});
        trials.Properties.VariableUnits={'s','s','int','int','s','s','int'};
        trials.Properties.VariableDescriptions={'first target hold time','go cue time','number of targets','number of targets attempted','first target hold time','outer target hold time','target number'};

    elseif db_version==1
        % *  Version 1 (0x01)
        % * ----------------
        % *  Version 1 includes a center target on and center target hold state, along with possible bumps
        % * byte   0: uchar => number of bytes to be transmitted
        % * byte   1: uchar => databurst version number (in this case: 0)
        % * byte   2 to 4: uchar => task code ('TRT')
        % * byte   5: uchar => model version major
        % * byte   6: uchar => model version minor
        % * bytes  7 to  8: short => model version micro
        % * bytes  9 to  12: float => x offset
        % * bytes 13 to 16: float => y offset
        % * bytes 17 to 20: float => target_size
        % * bytes 21 to 24: float => workspace number
        % * byte  25 : uchar => did bump?
        % * bytes 26 to 29: float => bump peak hold time
        % * bytes 30 to 33: float => bump rise time
        % * bytes 34 to 37: float => bump magnitude
        % * bytes 38 to 41: float => bump direction
        % * bytes 42 to 42+(N+1)*8: where N is the number of targets, contains 8 bytes per 
        % *      target representing two single-precision floating point numbers in 
        % *      little-endian format represnting the x and y position of the center of 
        % *      the target. This also includes the first, center target
        hdrSize=42;
        numTgt = (cds.databursts.db(1)-hdrSize)/8;

        ctHoldList=     nan(numTrials,1);
        targStartList=  nan(numTrials,1);
        goCueList=      nan(numTrials,numTgt);
        numTgts=        numTgt*ones(numTrials,1);
        numAttempted=   nan(numTrials,1);
        xOffsets=       nan(numTrials,1); 
        yOffsets=       nan(numTrials,1);
        tgtSizes=       nan(numTrials,1);
        wsnums=         nan(numTrials,1);
        bumpTimesList=  nan(numTrials,1);
        bumpDirList=    nan(numTrials,1);
        for trial = 1:numel(times.startTime)
            % Find databurst associated with startTime
            dbidx = find(cds.databursts.ts > times.startTime(trial) & cds.databursts.ts < times.endTime(trial));
            if length(dbidx) > 1
                warning('trt_trial_table: multiple databursts @ t = %.3f, using first:%d',times.startTime(trial),trial);
                dbidx = dbidx(1);
            elseif isempty(dbidx)
                warning('trt_trial_table: no/deleted databurst @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            end
            if (cds.databursts.db(dbidx,1)-hdrSize)/8 ~= numTgt
                %catch weird/corrupt databursts with different numbers of targets
                warning('trt_trial_table: Inconsistent number of targets @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            end

            % CT hold start times
            idxCTHold = find(ctHoldTimes > times.startTime(trial) & ctHoldTimes < times.endTime(trial));
            %identify trials with corrupt codes that might end up with extra center holds
            if isempty(idxCTHold)
                warning('trt_trial_table: No center hold @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            elseif length(idxCTHold) > 1
                warning('trt_trial_table: Multiple center hold @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            else
                ctHold = ctHoldTimes(idxCTHold);
            end

            % Bump times
            idxBump = find(bumpTimes > times.startTime(trial) & bumpTimes < times.endTime(trial));
            %identify trials with corrupt codes that might end up with extra bumps
            if isempty(idxBump)
                bumpTime = NaN;
            elseif length(idxCTHold) > 1
                warning('trt_trial_table: Multiple bump times @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            else
                bumpTime = bumpTimes(idxBump);
            end
            
            % Go cues
            idxGo = find(goCues > times.startTime(trial) & goCues < times.endTime(trial));

            %get the codes and times for the go cues
            goCue = nan(1,numTgt);
            if isempty(idxGo)
                tgtsAttempted = 0;
            else
                tgtsAttempted = length(idxGo);
            end
            if tgtsAttempted>0
                goCue(1:tgtsAttempted)=goCues(idxGo);
            end
            % extract first go cue
            targStart = goCue(1);

            %identify trials with corrupt end codes that might end up with extra
            %targets
            if length(idxGo) > numTgt
                warning('trt_trial_table: Inconsistent number of targets @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            end
            %find target centers
            ctr=bytes2float(cds.databursts.db(dbidx,hdrSize+1:end));
            % Offsets, target size
            xOffset = bytes2float(cds.databursts.db(dbidx,10:13));
            yOffset = bytes2float(cds.databursts.db(dbidx,14:17));
            tgtSize = bytes2float(cds.databursts.db(dbidx,18:21));
            wsnum = bytes2float(cds.databursts.db(dbidx,22:25));
            if isnan(bumpTime)
                bumpDir = NaN;
            else
                bumpDir = bytes2float(cds.databursts.db(dbidx,39:42));
            end

            % Build arrays
            ctHoldList(trial)=          ctHold;             % start time of center hold
            bumpTimesList(trial)=       bumpTime;           % time of bump
            targStartList(trial)=       targStart;          % time of first target onset
            goCueList(trial,:)=         goCue;              % time stamps of go_cue(s)
            numTgts(trial)=             numTgt;             % max number of targets
            numAttempted(trial)=        tgtsAttempted;      % number of targets for which a go cue was given
            xOffsets(trial)=            xOffset;            % x offset
            yOffsets(trial)=            yOffset;            % y offset
            tgtSizes(trial)=            tgtSize;            % target size
            wsnums(trial)=              wsnum;              % workspace number
            bumpDirList(trial)=         bumpDir;            % bump direction
            tgtCtrs(trial,:)=           ctr;                %center positions of the targets
        end

        trials=table(ctHoldList,bumpTimesList,targStartList,goCueList,numTgts,numAttempted,xOffsets,yOffsets,tgtSizes,wsnums,bumpDirList,tgtCtrs,...
                    'VariableNames',{'ctHoldTime','bumpTime','targetStartTime','goCueTime','numTgt','numAttempted','xOffset','yOffset','tgtSize','spaceNum','bumpDir','tgtCtr'});
        trials.Properties.VariableUnits={'s','s','s','s','int','int','cm','cm','cm','int','rad','cm'};
        trials.Properties.VariableDescriptions={'time of center hold start','time of bump','first target go cue time','go cue time','number of targets','number of targets attempted','x offset','y offset','target size','workspace number','bump direction','target center position'};
    else
        error('rw_trial_table_hdr:BadDataburstVersion',['Trial table parsing not implemented for databursts with version: ', num2str(db_version)])
    end
    if corruptDB==1
        cds.addProblem('There are corrupt databursts with more targets than expected. These have been skipped, but this frequently relates to errors in trial table parsting with the RW task')
    end
    trials=[times,trials];
    trials.Properties.Description='Trial table for the RW task';
    %cds.setField('trials',trials)
    set(cds,'trials',trials)
    evntData=loggingListenerEventData('getRWTaskTable',[]);
    notify(cds,'ranOperation',evntData)
end
