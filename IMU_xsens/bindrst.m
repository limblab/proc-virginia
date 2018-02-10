function[IMU] = bindrst(IMU)

rstall = [];

for ii = 1:size(IMU,2)
    rsts = find(IMU(ii).rst);
    rstall = [rstall; rsts];
end

rstall = unique(rstall);

for ii = 1:size(IMU,2)
    
    rstv = [];
    
    for j = 1:length(rstall)
        if IMU(ii).yw(rstall(j))==0
            rstv = [rstv, rstall(j)];
        elseif IMU(ii).yw(rstall(j)+1)==0
            rstv = [rstv, rstall(j)+1];
        else
            [~,indminyw] = min(abs([IMU(ii).yw(rstall(j)),IMU(ii).yw(rstall(j)+1)]));
            minrst = [rstall(j),rstall(j)+1];
            rstv = [rstv, minrst(indminyw)];
        end
    end
    
    rstv = [rstv, length(IMU(ii).yw)+1];
    
    IMU(ii).rstb = [];
    
    IMU(ii).rstb.yw = IMU(ii).yw;
    IMU(ii).rstb.pt = IMU(ii).pt;
    IMU(ii).rstb.rl = IMU(ii).rl;
    
    IMU(ii).rstb.q.yw = IMU(ii).q.yw;
    IMU(ii).rstb.q.pt = IMU(ii).q.pt;
    IMU(ii).rstb.q.rl = IMU(ii).q.rl;
    
    for jj = 1:length(rstv)-1
        IMU(ii).rstb.yw(rstv(jj):rstv(jj+1)-1) = IMU(ii).yw(rstv(jj):rstv(jj+1)-1)-(IMU(ii).yw(rstv(jj))-IMU(ii).rstb.yw(rstv(jj)-1));
        IMU(ii).rstb.pt(rstv(jj):rstv(jj+1)-1) = IMU(ii).pt(rstv(jj):rstv(jj+1)-1)-(IMU(ii).pt(rstv(jj))-IMU(ii).rstb.pt(rstv(jj)-1));
        IMU(ii).rstb.rl(rstv(jj):rstv(jj+1)-1) = IMU(ii).rl(rstv(jj):rstv(jj+1)-1)-(IMU(ii).rl(rstv(jj))-IMU(ii).rstb.rl(rstv(jj)-1));
           
        IMU(ii).rstb.q.yw(rstv(jj):rstv(jj+1)-1) = IMU(ii).q.yw(rstv(jj):rstv(jj+1)-1)-(IMU(ii).q.yw(rstv(jj))-IMU(ii).rstb.q.yw(rstv(jj)-1));
        IMU(ii).rstb.q.pt(rstv(jj):rstv(jj+1)-1) = IMU(ii).q.pt(rstv(jj):rstv(jj+1)-1)-(IMU(ii).q.pt(rstv(jj))-IMU(ii).rstb.q.pt(rstv(jj)-1));
        IMU(ii).rstb.q.rl(rstv(jj):rstv(jj+1)-1) = IMU(ii).q.rl(rstv(jj):rstv(jj+1)-1)-(IMU(ii).q.rl(rstv(jj))-IMU(ii).rstb.q.rl(rstv(jj)-1));
    end
end
end