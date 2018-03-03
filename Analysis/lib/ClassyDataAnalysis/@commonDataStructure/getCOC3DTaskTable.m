function getCOC3DTaskTable(cds,times)

%this is a method function for the common_data_structure (cds) class, and
%should be located in a folder '@common_data_structure' with the class
%definition file and other method files
%
%cds.getCOC3DTaskTable(times)
% returns no value, instead it populates the trials field
%of the cds assuming the task is a two-workspace random target task. Takes a single
%input:times, which is a table with 4 columns: number, startTime,
%endTime, and result. These times define the start and stop of trials
%as indicated by the state words for trial start and trial end. the
%result code will be a character 'R':reward 'A':abort 'F':fail
%'I':incomplete.

corruptDB=0;

numTrials = length(times.number);

wordStOn = hex2dec('30');
stOnTimes =  cds.words.ts((cds.words.word) == wordStOn);
wordStHold = hex2dec('A0');
stHoldTimes = cds.words.ts(bitand(hex2dec('f0'), cds.words.word) == wordStHold);
wordGo = hex2dec('31');
goCues =  cds.words.ts((cds.words.word) == wordGo);
wordStLeave = hex2dec('40');
stLeaveTimes =  cds.words.ts((cds.words.word) == wordStLeave);

wordOtHold = hex2dec('A1');
otHoldTimes = cds.words.ts((cds.words.word) == wordOtHold);
wordGoBack = hex2dec('32');
goBackCues =  cds.words.ts((cds.words.word) == wordGoBack);
wordOtLeave = hex2dec('41');
otLeaveTimes =  cds.words.ts((cds.words.word) == wordOtLeave);

wordFTHold = hex2dec('A2');
ftHoldTimes = cds.words.ts((cds.words.word) == wordFTHold);

%check DB version number and run appropriate parsing code
db_version=cds.databursts.db(1,2);

    if db_version == 0
        %  * Version 0 (0x00)
        %  * ----------------
        %  * byte   0: uchar => number of bytes to be transmitted
        %  * byte   1: uchar => databurst version number (in this case: 0)
        %  * byte   2 to 4: uchar => task code ('COC')
        %  * byte   5: uchar => model version major
        %  * byte   6: uchar => model version minor
        %  * bytes  7 to  8: short => model version micro
        %  * bytes  9 to 12: float => outer target number. targets
        %  *          are numbered 1-7, counter-clockwise, starting from bottom right
        %  * bytes 13 to 16: float => start (center) target hold time
        %  * bytes 17 to 20: float => outer target hold time 
        %  * bytes 21 to 24: float => final (center) target hold time
        %  * byte  25: uchar => IMU reset on this trial
        %  */

        stOnList=           nan(numTrials,1);
        stHoldList=         nan(numTrials,1);
        otHoldList=         nan(numTrials,1);
        ftHoldList=         nan(numTrials,1);
        goCueList=          nan(numTrials,1);
        goBackCueList=      nan(numTrials,1);
        stLeaveList=        nan(numTrials,1);
        otLeaveList=        nan(numTrials,1);
        stHoldTimeList=     nan(numTrials,1);
        otHoldTimeList=     nan(numTrials,1);
        ftHoldTimeList=     nan(numTrials,1);
        resetList=          nan(numTrials,1);
        otNumList=          nan(numTrials,1);

        for trial = 1:numel(times.startTime)
            % Find databurst associated with startTime
            dbidx = find(cds.databursts.ts > times.startTime(trial) & cds.databursts.ts < times.endTime(trial));
            if length(dbidx) > 1
                warning('coc3d_trial_table: multiple databursts @ t = %.3f, using first:%d',times.startTime(trial),trial);
                dbidx = dbidx(1);
            elseif isempty(dbidx)
                warning('coc3d_trial_table: no/deleted databurst @ t = %.3f, skipping trial:%d',times.startTime(trial),trial);
                corruptDB=1;
                continue;
            end

            % Start target on times
            idxStOn = find(stOnTimes > times.startTime(trial) & stOnTimes < times.endTime(trial),1,'first');
            if isempty(idxStOn)
                stOn = NaN;
            else
                stOn = stOnTimes(idxStOn);
            end
            
            % Start target hold times
            idxStHold = find(stHoldTimes > times.startTime(trial) & stHoldTimes < times.endTime(trial),1,'first');
            if isempty(idxStHold)
                stHold = NaN;
            else
                stHold = stHoldTimes(idxStHold);
            end
  
            % Go cues
            idxGo = find(goCues > times.startTime(trial) & goCues < times.endTime(trial));
            if isempty(idxGo)
                goCue = NaN;
            else
                goCue = goCues(idxGo);
            end
            
            % Start target leave times
            idxStLeave = find(stLeaveTimes > times.startTime(trial) & stLeaveTimes < times.endTime(trial),1,'first');
            if isempty(idxStLeave)
                stLeave = NaN;
            else
                stLeave = stLeaveTimes(idxStLeave);
            end
            
            % Outer target hold times
            idxOtHold = find(otHoldTimes > times.startTime(trial) & otHoldTimes < times.endTime(trial),1,'first');
            if isempty(idxOtHold)
                otHold = NaN;
            else
                otHold = otHoldTimes(idxOtHold);
            end
            
            % Go cues
            idxGoBack = find(goBackCues > times.startTime(trial) & goBackCues < times.endTime(trial));
            if isempty(idxGoBack)
                goBackCue = NaN;
            else
                goBackCue = goBackCues(idxGoBack);
            end
            
            % Outer target leave times
            idxOtLeave = find(otLeaveTimes > times.startTime(trial) & otLeaveTimes < times.endTime(trial),1,'first');
            if isempty(idxOtLeave)
                otLeave = NaN;
            else
                otLeave = otLeaveTimes(idxOtLeave);
            end
            
            % Final target hold times
            idxFtHold = find(ftHoldTimes > times.startTime(trial) & ftHoldTimes < times.endTime(trial),1,'first');
            if isempty(idxFtHold)
                ftHold = NaN;
            else
                ftHold = ftHoldTimes(idxFtHold);
            end
            % Get outer target number
            otNum = bytes2float(cds.databursts.db(dbidx,10:13));
            
            % Get hold times
            stHoldTime = bytes2float(cds.databursts.db(dbidx,14:17));
            otHoldTime = bytes2float(cds.databursts.db(dbidx,18:21));
            ftHoldTime = bytes2float(cds.databursts.db(dbidx,22:25));
            
            % Get IMU reset times
            resetTimes = cds.databursts.db(dbidx,26);

            % Build arrays
            stOnList(trial,:)=              stOn;              % time of start target onset
            stHoldList(trial,:)=            stHold;            % time of start target hold
            goCueList(trial,:)=             goCue;             % time stamps of go cue
            stLeaveList(trial,:)=           stLeave;           % time of start target leave
            otHoldList(trial,:)=            otHold;            % time of outer target hold
            goBackCueList(trial,:)=         goBackCue;         % time stamps of go back cue
            otLeaveList(trial,:)=           otLeave;           % time of start target leave
            ftHoldList(trial,:)=            ftHold;            % time of final target hold

            stHoldTimeList(trial,:)=        stHoldTime;        % start target hold time
            otHoldTimeList(trial,:)=        otHoldTime;        % outer target hold time
            ftHoldTimeList(trial,:)=        ftHoldTime;        % final target hold time

            otNumList(trial,:)=             otNum;             % outer target number
            
            resetList(trial,:)=             resetTimes;        % IMU reset trigger

        end

        trials=table(stOnList,stHoldList,goCueList,stLeaveList,otHoldList,goBackCueList,...
            otLeaveList,ftHoldList,otNumList,resetList,stHoldTimeList,otHoldTimeList,ftHoldTimeList,...
            'VariableNames',{'stOn','stHold','goCue','stLeave','otHold','goBackCue',...
            'otLeave','ftHold','otNum','IMUreset','stHoldTime','otHoldTime','ftHoldTime'});
        trials.Properties.VariableUnits={'s','s','s','s','s','s','s','s','int','int','s','s','s'};
        trials.Properties.VariableDescriptions={'start target on','start target hold','go cue',...
            'start target leave','outer target hold','go back cue','outer target leave',...
            'final target hold','outer target number','IMU reset trigger','start target hold time','outer target hold time','final target hold time'};

    else
        trials = [];
    end

    if corruptDB==1
        cds.addProblem('There are corrupt databursts with more targets than expected. These have been skipped, but this frequently relates to errors in trial table parsting with the RW task')
    end
    trials=[times,trials];
    trials.Properties.Description='Trial table for the COC3D task';
    %cds.setField('trials',trials)
    set(cds,'trials',trials)
    evntData=loggingListenerEventData('getCOC3DTaskTable',[]);
    notify(cds,'ranOperation',evntData)
end