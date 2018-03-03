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
    goCues =  cds.words.ts((cds.words.word) == wordGo);

    wordCTHold = hex2dec('30');
    ctHoldTimes = cds.words.ts(bitand(hex2dec('f0'), cds.words.word) == wordCTHold);

    wordTargHold = hex2dec('A1');
    targMask=(cds.words.word) == wordTargHold;
    otHoldTimes = cds.words.ts(targMask);

    %check DB version number and run appropriate parsing code
    % DB version 0 has 25 bytes before target position
    db_version=cds.databursts.db(1,2);

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

        CtStartList=        nan(numTrials,1);
        OtHoldList=         nan(numTrials,1);
        goCueList=          nan(numTrials,numTgt);
        numTgts=            numTgt*ones(numTrials,1);
        numAttempted=       nan(numTrials,1);
        ftHoldTimeList=     nan(numTrials,1);
        otHoldTimeList=     nan(numTrials,1);

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

            % Central target on times
            idxCtHold = find(ctHoldTimes > times.startTime(trial) & ctHoldTimes < times.endTime(trial),1,'first');
            %identify trials with corrupt codes that might end up with extra
            %targets
            if isempty(idxCtHold)
                CtStart = NaN;
            else
                CtStart = ctHoldTimes(idxCtHold);
            end
            
            % Outer target hold times
            idxOtHold = find(otHoldTimes > times.startTime(trial) & otHoldTimes < times.endTime(trial),1,'first');
            %identify trials with corrupt codes that might end up with extra
            %targets
            if isempty(idxOtHold)
                OtHold = NaN;
            else
                OtHold = otHoldTimes(idxOtHold);
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

            %identify trials with corrupt end codes that might end up with extra
            %targets
            if length(idxGo) > numTgt
                warning('trt_trial_table: Inconsistent number of targets @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            end
            
            % Get hold times
            ftHoldTime = bytes2float(cds.databursts.db(dbidx,14:17));
            targHoldTime = bytes2float(cds.databursts.db(dbidx,18:21));

            % Get target numbers
            tgtnum=bytes2float(cds.databursts.db(dbidx,hdrSize+1:end));

            % Build arrays
            CtStartList(trial,:)=       CtStart;            % time of first target onset
            OtHoldList(trial,:)=        OtHold;             % time of outer target hold
            goCueList(trial,:)=         goCue;              % time stamps of go_cue(s)
            numTgts(trial)=             numTgt;             % max number of targets
            numAttempted(trial,:)=      tgtsAttempted;      % ?
            ftHoldTimeList(trial,:)=    ftHoldTime;         % first target hold time
            otHoldTimeList(trial,:)=    targHoldTime;       % outer target hold time
            tgt(trial,:)=               tgtnum;             %target number
        end

        trials=table(CtStartList,goCueList,OtHoldList,numTgts,numAttempted,ftHoldTimeList,otHoldTimeList,tgt,...
            'VariableNames',{'CTStartTime','goCueTime','OTHoldTime','numTgt','numAttempted','ftHoldTime','targHoldTime','tgtnum'});
        trials.Properties.VariableUnits={'s','s','s','int','int','s','s','int'};
        trials.Properties.VariableDescriptions={'center target on time','go cue time','outer target hold time','number of targets','number of targets attempted','first target hold time','outer target hold time','target number'};

    end

    if corruptDB==1
        cds.addProblem('There are corrupt databursts with more targets than expected. These have been skipped, but this frequently relates to errors in trial table parsting with the RW task')
    end
    trials=[times,trials];
    trials.Properties.Description='Trial table for the RT3D task';
    %cds.setField('trials',trials)
    set(cds,'trials',trials)
    evntData=loggingListenerEventData('getRT3DTaskTable',[]);
    notify(cds,'ranOperation',evntData)
end
