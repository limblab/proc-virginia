function[JA] = getJAsegm(IMU,JA,tJA)

nJA = length(tJA)/2;
vJA = 1:2:nJA*2;

for ii = 1:nJA
    [~,ix1] = min(abs(IMU(1).stimem-tJA(vJA(ii))));
    [~,ix2] = min(abs(IMU(1).stimem-tJA(vJA(ii)+1)));
    
    for k = 1:size(IMU,2)
        JA(k).(['S',num2str(ii)]).time = (IMU(1).stimem(ix1:ix2)-IMU(1).stimem(ix1))';
        
        JA(k).(['S',num2str(ii)]).rlg = JA(k).rlg(ix1:ix2);
        JA(k).(['S',num2str(ii)]).ptg = JA(k).ptg(ix1:ix2);
        JA(k).(['S',num2str(ii)]).ywg = JA(k).ywg(ix1:ix2);
    end
    
    for j = 1:size(IMU,2)-1
        JA(j).(['S',num2str(ii)]).rl = JA(j).rl(ix1:ix2);
        JA(j).(['S',num2str(ii)]).pt = JA(j).pt(ix1:ix2);
        JA(j).(['S',num2str(ii)]).yw = JA(j).yw(ix1:ix2);
    end
end
end