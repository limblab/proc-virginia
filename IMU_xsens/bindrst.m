function[IMU] = bindrst(IMU)
rstv = find(IMU(1).rst);
for ii = 1:size(IMU,2)
    IMU(ii).rstb = [];
    IMU(ii).rstb.yw = IMU(ii).yw;
    IMU(ii).rstb.pt = IMU(ii).pt;
    IMU(ii).rstb.rl = IMU(ii).rl;
    
    for jj = 1:length(rstv)-1
        IMU(ii).rstb.yw(rstv(jj):rstv(jj+1)-1) = IMU(ii).yw(rstv(jj):rstv(jj+1)-1)+(IMU(ii).yw(rstv(jj)-1)-IMU(ii).yw(rstv(jj)));
        IMU(ii).rstb.pt(rstv(jj):rstv(jj+1)-1) = IMU(ii).pt(rstv(jj):rstv(jj+1)-1)+(IMU(ii).pt(rstv(jj)-1)-IMU(ii).pt(rstv(jj)));
        IMU(ii).rstb.rl(rstv(jj):rstv(jj+1)-1) = IMU(ii).rl(rstv(jj):rstv(jj+1)-1)+(IMU(ii).rl(rstv(jj)-1)-IMU(ii).rl(rstv(jj)));
    end
end
end