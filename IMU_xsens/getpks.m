function[JA] = getpks(JA)

for ii = 1:size(JA,2)-1
    [~, rlpks] = findpeaks((JA(ii).rl), 'minpeakheight', mean(JA(ii).rl)+std(JA(ii).rl)/2,'minpeakdistance',100);
    [~, ptpks] = findpeaks((JA(ii).pt), 'minpeakheight', mean(JA(ii).pt)+std(JA(ii).pt)/2,'minpeakdistance',100);
    [~, ywpks] = findpeaks((JA(ii).yw), 'minpeakheight', mean(JA(ii).yw)+std(JA(ii).yw)/2,'minpeakdistance',100);
    
    JA(ii).pks.trl = JA(1).time(rlpks);
    JA(ii).pks.tpt = JA(1).time(ptpks);
    JA(ii).pks.tyw = JA(1).time(ywpks);
    
    JA(ii).pks.rl = JA(ii).rl(rlpks);
    JA(ii).pks.pt = JA(ii).pt(ptpks);
    JA(ii).pks.yw = JA(ii).yw(ywpks);
    
    JA(ii).pks.mrl = mean(JA(ii).rl(rlpks));
    JA(ii).pks.mpt = mean(JA(ii).pt(ptpks));
    JA(ii).pks.myw = mean(JA(ii).yw(ywpks));
    
    JA(ii).pks.stdrl = std(JA(ii).rl(rlpks));
    JA(ii).pks.stdpt = std(JA(ii).pt(ptpks));
    JA(ii).pks.stdyw = std(JA(ii).yw(ywpks));   
end

for ii = 1:size(JA,2)
    [~, rlgpks] = findpeaks((JA(ii).rlg), 'minpeakheight', mean(JA(ii).rlg)+std(JA(ii).rlg)/2,'minpeakdistance',100);
    [~, ptgpks] = findpeaks((JA(ii).ptg), 'minpeakheight', mean(JA(ii).ptg)+std(JA(ii).ptg)/2,'minpeakdistance',100);
    [~, ywgpks] = findpeaks((JA(ii).ywg), 'minpeakheight', mean(JA(ii).ywg)+std(JA(ii).ywg)/2,'minpeakdistance',100);
    
    JA(ii).pks.trlg = JA(1).time(rlgpks);
    JA(ii).pks.tptg = JA(1).time(ptgpks);
    JA(ii).pks.tywg = JA(1).time(ywgpks);
    
    JA(ii).pks.rlg = JA(ii).rlg(rlgpks);
    JA(ii).pks.ptg = JA(ii).ptg(ptgpks);
    JA(ii).pks.ywg = JA(ii).ywg(ywgpks);
    
    JA(ii).pks.mrlg = mean(JA(ii).rlg(rlgpks));
    JA(ii).pks.mptg = mean(JA(ii).ptg(ptgpks));
    JA(ii).pks.mywg = mean(JA(ii).ywg(ywgpks));
    
    JA(ii).pks.stdrlg = std(JA(ii).rlg(rlgpks));
    JA(ii).pks.stdptg = std(JA(ii).ptg(ptgpks));
    JA(ii).pks.stdywg = std(JA(ii).ywg(ywgpks));
end

j = 1;
seg = ['S',num2str(j)];

while isfield(JA,seg)
    for ii = 1:size(JA,2)-1
        [~, rlpks] = findpeaks((JA(ii).(seg).rl), 'minpeakheight', mean(JA(ii).(seg).rl)+std(JA(ii).(seg).rl)/2,'minpeakdistance',100);
        [~, ptpks] = findpeaks((JA(ii).(seg).pt), 'minpeakheight', mean(JA(ii).(seg).pt)+std(JA(ii).(seg).pt)/2,'minpeakdistance',100);
        [~, ywpks] = findpeaks((JA(ii).(seg).yw), 'minpeakheight', mean(JA(ii).(seg).yw)+std(JA(ii).(seg).yw)/2,'minpeakdistance',100);
        
        JA(ii).(seg).pks.trl = JA(ii).(seg).time(rlpks);
        JA(ii).(seg).pks.tpt = JA(ii).(seg).time(ptpks);
        JA(ii).(seg).pks.tyw = JA(ii).(seg).time(ywpks);
        
        JA(ii).(seg).pks.rl = JA(ii).(seg).rl(rlpks);
        JA(ii).(seg).pks.pt = JA(ii).(seg).pt(ptpks);
        JA(ii).(seg).pks.yw = JA(ii).(seg).yw(ywpks);
        
        JA(ii).(seg).pks.mrl = mean(JA(ii).(seg).rl(rlpks));
        JA(ii).(seg).pks.mpt = mean(JA(ii).(seg).pt(ptpks));
        JA(ii).(seg).pks.myw = mean(JA(ii).(seg).yw(ywpks));
        
        JA(ii).(seg).pks.stdrl = std(JA(ii).(seg).rl(rlpks));
        JA(ii).(seg).pks.stdpt = std(JA(ii).(seg).pt(ptpks));
        JA(ii).(seg).pks.stdyw = std(JA(ii).(seg).yw(ywpks));
    end
    for ii = 1:size(JA,2)
        [~, rlgpks] = findpeaks((JA(ii).(seg).rlg), 'minpeakheight', mean(JA(ii).(seg).rlg)+std(JA(ii).(seg).rlg)/2,'minpeakdistance',100);
        [~, ptgpks] = findpeaks((JA(ii).(seg).ptg), 'minpeakheight', mean(JA(ii).(seg).ptg)+std(JA(ii).(seg).ptg)/2,'minpeakdistance',100);
        [~, ywgpks] = findpeaks((JA(ii).(seg).ywg), 'minpeakheight', mean(JA(ii).(seg).ywg)+std(JA(ii).(seg).ywg)/2,'minpeakdistance',100);
        
        JA(ii).(seg).pks.trlg = JA(ii).(seg).time(rlgpks);
        JA(ii).(seg).pks.tptg = JA(ii).(seg).time(ptgpks);
        JA(ii).(seg).pks.tywg = JA(ii).(seg).time(ywgpks);
        
        JA(ii).(seg).pks.rlg = JA(ii).(seg).rlg(rlgpks);
        JA(ii).(seg).pks.ptg = JA(ii).(seg).ptg(ptgpks);
        JA(ii).(seg).pks.ywg = JA(ii).(seg).ywg(ywgpks);
        
        JA(ii).(seg).pks.mrlg = mean(JA(ii).(seg).rlg(rlgpks));
        JA(ii).(seg).pks.mptg = mean(JA(ii).(seg).ptg(ptgpks));
        JA(ii).(seg).pks.mywg = mean(JA(ii).(seg).ywg(ywgpks));
        
        JA(ii).(seg).pks.stdrlg = std(JA(ii).(seg).rlg(rlgpks));
        JA(ii).(seg).pks.stdptg = std(JA(ii).(seg).ptg(ptgpks));
        JA(ii).(seg).pks.stdywg = std(JA(ii).(seg).ywg(ywgpks));
    end
    j = j+1;
    seg = ['S',num2str(j)];
end
end