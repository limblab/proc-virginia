function[JA] = getjointangles(IMU,JA,oritype,filt)

joints = {'elb','sho'};
nelb = find(strcmp({IMU.place},'elb'));
nsho = find(strcmp({IMU.place},'sho'));
nback = find(strcmp({IMU.place},'back'));

for ii = 1:length(IMU(1).stime)
    for jj = 1:3
        if strcmp(oritype,'eul') && ~filt
            JA(jj).Rgs = ypr2Rmat(IMU(jj).yw(ii),IMU(jj).pt(ii),IMU(jj).rl(ii)); % Sensor to global matrix
        elseif strcmp(oritype,'eul') && filt
            JA(jj).Rgs = ypr2Rmat(IMU(jj).filt.yw(ii),IMU(jj).filt.pt(ii),IMU(jj).filt.rl(ii));
        elseif strcmp(oritype,'quat') && ~filt
            JA(jj).Rgs = quat2Rmat(IMU(jj).q.q0(ii),IMU(jj).q.q1(ii),IMU(jj).q.q2(ii),IMU(jj).q.q3(ii));
        else
            JA(jj).Rgs = quat2Rmat(IMU(jj).filt.q.q0(ii),IMU(jj).filt.q.q1(ii),IMU(jj).filt.q.q2(ii),IMU(jj).filt.q.q3(ii));
        end
        [JA(jj).ywg(ii),JA(jj).ptg(ii),JA(jj).rlg(ii)] = Rmat2ypr(JA(jj).Rgs*JA(jj).Rsb); % Reconstructed global IMU angles
    end
    
    JA(1).Rbb(:,:,ii) = inv(JA(nelb).Rgs*JA(nelb).Rsb)*(JA(nsho).Rgs*JA(nsho).Rsb); % Segment to segment matrix
    JA(2).Rbb(:,:,ii) = inv(JA(nback).Rgs*JA(nback).Rsb)*(JA(nsho).Rgs*JA(nsho).Rsb); % Segment to segment matrix
    
    for kk = 1:2
        [JA(kk).yw(ii),JA(kk).pt(ii),JA(kk).rl(ii)] = Rmat2ypr(JA(kk).Rbb(:,:,ii)); % Reconstructed joint angles
        JA(kk).place = joints{kk};
    end
    
end