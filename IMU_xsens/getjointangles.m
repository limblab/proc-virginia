function[JA] = getjointangles(IMU,JA,oritype,filt,rst)

joints = {'elb','sho'};
for ii = 1:length(joints)
    JA(ii).joint = joints{ii};
end

nelb = find(strcmp({IMU.place},'elb'));
nsho = find(strcmp({IMU.place},'sho'));
nback = find(strcmp({IMU.place},'back'));

for ii = 1:length(JA(1).time)
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
    
    for kk = 1:size(IMU,2)-1
        [JA(kk).yw(ii),JA(kk).pt(ii),JA(kk).rl(ii)] = Rmat2ypr(JA(kk).Rbb(:,:,ii)); % Reconstructed joint angles
        JA(kk).place = joints{kk};
    end         
end

if rst
    for kk = 1:size(IMU,2)-1
        JA(kk).yw = JA(kk).yw-(mean(JA(kk).yw(JA(1).ixp.ix1:JA(1).ixp.ix2)));
        JA(kk).pt = JA(kk).pt-(mean(JA(kk).pt(JA(1).ixp.ix1:JA(1).ixp.ix2)));
        JA(kk).rl = JA(kk).rl-(mean(JA(kk).rl(JA(1).ixp.ix1:JA(1).ixp.ix2)));
    end
    
    JA(1).rl = JA(1).rl-90;
    
    for ii = 1:size(IMU,2)
        JA(ii).ywg = JA(ii).ywg-(mean(JA(ii).ywg(JA(1).ixp.ix1:JA(1).ixp.ix2)));
        JA(ii).ptg = JA(ii).ptg-(mean(JA(ii).ptg(JA(1).ixp.ix1:JA(1).ixp.ix2)));
        JA(ii).rlg = JA(ii).rlg-(mean(JA(ii).rlg(JA(1).ixp.ix1:JA(1).ixp.ix2)));
    end
end

JA(1).ywd = JA(nelb).ywg-JA(nsho).ywg-JA(nback).ywg;
JA(1).ptd = JA(nelb).ptg-JA(nsho).ptg-JA(nback).ptg;
JA(1).rld = JA(nelb).rlg-JA(nsho).rlg-JA(nback).rlg;

JA(2).ywd = JA(nsho).ywg-JA(nback).ywg;
JA(2).ptd = JA(nsho).ptg-JA(nback).ptg;
JA(2).rld = JA(nsho).rlg-JA(nback).rlg;

end
          