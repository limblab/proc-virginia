function[JA] = getjointangles(IMU,JA,oritype,filt,rst,correct)
% Obtains body segment to body segment transformation matrix (Rbb), joint
% angles (rl, pt, yw), and reconstructed IMU body referenced angles (rlg,
% ptg, ywg)

% JA: joint angle data structure
% IMU: IMU data structure
% oritype: which orientation representation was used 'quat' or 'eul'
% filt: whether to use filtered data
% rst: whether to detrend data and bring to OpenSim reference angles
% correct: whether to correct Rsb for flexion angle offset

% Implemented from:
%    Palermo E, Rossi S, Marini F, Patanè F, Cappa P.
%    Experimental evaluation of accuracy and repeatability of a novel
%    body-to-sensor calibration procedure for inertial sensor-based gait analysis.
%    Measurement. 2014 Jun 1;52:145?55.

nelb = find(strcmp({IMU.place},'elb'));
nsho = find(strcmp({IMU.place},'sho'));
nback = find(strcmp({IMU.place},'back'));

if ~isempty(nelb) && ~isempty(nsho) && ~isempty(nback)
    joints = {'elb','sho'};
end
if ~isempty(nelb) && ~isempty(nsho) && isempty(nback)
    joints = {'elb'};
end
if isempty(nelb) && ~isempty(nsho) && ~isempty(nback)
    joints = {'sho'};
end

% Correct Rsb
if correct
    for ii = 1:size(IMU,2)
        if isfield(IMU,'acc_calib')
            accv = mean(IMU(ii).acc_calib(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
        else
            accv = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
        end
        JA(ii).RAA = vec2Rmat([0 accv(2:3)],[0 0 1]);
        %JA(ii).RFE = vec2Rmat([accv(1:2) 0],[0 0 1]);
        JA(ii).Rsb = JA(ii).RAA*JA(ii).Rsb;
    end
end

for ii = 1:length(JA(1).time)
    for jj = 1:size(IMU,2)
        % Get rotation matrix from sensor data
        if strcmp(oritype,'eul') && ~filt
            JA(jj).Rgs = ypr2Rmat(IMU(jj).yw(ii),IMU(jj).pt(ii),IMU(jj).rl(ii));
        elseif strcmp(oritype,'eul') && filt
%             if ii == 1
%                 JA(jj).Rgs = ypr2Rmat(180,0,0)*ypr2Rmat(IMU(jj).filt.yw(ii),IMU(jj).filt.pt(ii),IMU(jj).filt.rl(ii));
%             else
                JA(jj).Rgs = ypr2Rmat(IMU(jj).filt.yw(ii),IMU(jj).filt.pt(ii),IMU(jj).filt.rl(ii));
%             end
        elseif strcmp(oritype,'quat') && ~filt
            JA(jj).Rgs = quat2Rmat(IMU(jj).q.q0(ii),IMU(jj).q.q1(ii),IMU(jj).q.q2(ii),IMU(jj).q.q3(ii));  
        else
            JA(jj).Rgs = ypr2Rmat(IMU(jj).filt.q.yw(ii),IMU(jj).filt.q.pt(ii),IMU(jj).filt.q.rl(ii));
        end
        % Reconstruct body IMU angles
        [JA(jj).ywg(ii),JA(jj).ptg(ii),JA(jj).rlg(ii)] = Rmat2ypr(JA(jj).Rgs*JA(jj).Rsb);
    end
    
    % Body segment to body segment transformation matrix
    if ~isempty(nelb) && ~isempty(nsho) && ~isempty(nback)
        JA(1).Rbb(:,:,ii) = inv(JA(nelb).Rgs*JA(nelb).Rsb)*(JA(nsho).Rgs*JA(nsho).Rsb); % Elbow angles
        JA(2).Rbb(:,:,ii) = inv(JA(nsho).Rgs*JA(nsho).Rsb)*(JA(nback).Rgs*JA(nback).Rsb); % Shoulder angles
    elseif isempty(nback) && ~isempty(nsho) && ~isempty(nelb)
        JA(1).Rbb(:,:,ii) = inv(JA(nelb).Rgs*JA(nelb).Rsb)*(JA(nsho).Rgs*JA(nsho).Rsb); % Elbow angles
    elseif ~isempty(nback) && ~isempty(nsho) && isempty(nelb)
        JA(1).Rbb(:,:,ii) = inv(JA(nsho).Rgs*JA(nsho).Rsb)*(JA(nback).Rgs*JA(nback).Rsb); % Shoulder angles
    end
    
    % Reconstruct joint angles from Rbb
    for kk = 1:size(JA,2)-1
        [JA(kk).yw(ii),JA(kk).pt(ii),JA(kk).rl(ii)] = Rmat2ypr(JA(kk).Rbb(:,:,ii));
        JA(kk).place = joints{kk};
    end
end

% Get reference angles when Cerebus file recording
if isfield(IMU,'acc_calib')
    ang_calib = JA(1).ixp.ix1:JA(1).ixp.ix2;
    for hh = 1:length(ang_calib)
        for ii = 1:size(IMU,2)
            if strcmp(oritype,'eul') && isfield(IMU,'eul_calib')
                Rgs_c(:,:,ii) = ypr2Rmat(IMU(ii).eul_calib(ang_calib(hh),3),IMU(ii).eul_calib(ang_calib(hh),2),IMU(ii).eul_calib(ang_calib(hh),1));
            elseif strcmp(oritype,'quat') && isfield(IMU,'q_calib')
                Rgs_c(:,:,ii) = quat2Rmat(IMU(ii).q_calib(ang_calib(hh),1),IMU(ii).q_calib(ang_calib(hh),2),IMU(ii).q_calib(ang_calib(hh),3),IMU(jj).q_calib(ang_calib(hh),4));
            end
            [JA(ii).ywgc(hh),JA(ii).ptgc(hh),JA(ii).rlgc(hh)] = Rmat2ypr(Rgs_c(:,:,ii)*JA(ii).Rsb);
        end
        
        Rbb_e(:,:,hh) = inv(Rgs_c(:,:,nelb)*JA(nelb).Rsb)*(Rgs_c(:,:,nsho)*JA(nsho).Rsb); % Elbow angles
        Rbb_s(:,:,hh) = inv(Rgs_c(:,:,nsho)*JA(nsho).Rsb)*(Rgs_c(:,:,nback)*JA(nback).Rsb); % Shoulder angles
        
        [yw_ce(hh),pt_ce(hh),rl_ce(hh)] = Rmat2ypr(Rbb_e(:,:,hh));
        [yw_cs(hh),pt_cs(hh),rl_cs(hh)] = Rmat2ypr(Rbb_s(:,:,hh));
    end
end

% Remove mean from data and match to OpenSim reference
if rst && ~isfield(IMU,'time_calib')
    for kk = 1:size(IMU,2)-1
        JA(kk).yw = JA(kk).yw-mean(JA(kk).yw(JA(1).ixp.ix1:JA(1).ixp.ix2));
        JA(kk).pt = JA(kk).pt-mean(JA(kk).pt(JA(1).ixp.ix1:JA(1).ixp.ix2));
        JA(kk).rl = JA(kk).rl-mean(JA(kk).rl(JA(1).ixp.ix1:JA(1).ixp.ix2));
    end
        
    for ii = 1:size(IMU,2)
        JA(ii).ywg = JA(ii).ywg-mean(JA(ii).ywg(JA(1).ixp.ix1:JA(1).ixp.ix2));
        JA(ii).ptg = JA(ii).ptg-mean(JA(ii).ptg(JA(1).ixp.ix1:JA(1).ixp.ix2));
        JA(ii).rlg = JA(ii).rlg-mean(JA(ii).rlg(JA(1).ixp.ix1:JA(1).ixp.ix2));
    end
elseif rst && isfield(IMU,'acc_calib')
    JA(1).yw = JA(1).yw-(mean(yw_ce));
    JA(1).pt = JA(1).pt-(mean(pt_ce));
    JA(1).rl = JA(1).rl-(mean(rl_ce));
    
    JA(2).yw = JA(2).yw-(mean(yw_cs));
    JA(2).pt = JA(2).pt-(mean(pt_cs));
    JA(2).rl = JA(2).rl-(mean(rl_cs));
    
    for ii = 1:size(IMU,2)
        JA(ii).ywg = JA(ii).ywg-(mean(JA(ii).ywgc));
        JA(ii).ptg = JA(ii).ptg-(mean(JA(ii).ptgc));
        JA(ii).rlg = JA(ii).rlg-(mean(JA(ii).rlgc));
    end
    
    JA(1).ywc = yw_ce;
    JA(1).ptc = pt_ce;
    JA(1).rlc = rl_ce;
    
    JA(2).ywc = yw_cs;
    JA(2).ptc = pt_cs;
    JA(2).rlc = rl_cs;
    
end

% Obtain joint angles by the difference of IMU orientation data
if ~isempty(nelb) && ~isempty(nsho) && ~isempty(nback)
    JA(1).ywd = JA(nelb).ywg-JA(nsho).ywg-JA(nback).ywg;
    JA(1).ptd = JA(nelb).ptg-JA(nsho).ptg-JA(nback).ptg;
    JA(1).rld = JA(nelb).rlg-JA(nsho).rlg-JA(nback).rlg;
    
    JA(2).ywd = JA(nsho).ywg-JA(nback).ywg;
    JA(2).ptd = JA(nsho).ptg-JA(nback).ptg;
    JA(2).rld = JA(nsho).rlg-JA(nback).rlg;
elseif isempty(nelb) && ~isempty(nsho) && ~isempty(nback)
    JA(1).ywd = JA(nsho).ywg-JA(nback).ywg;
    JA(1).ptd = JA(nsho).ptg-JA(nback).ptg;
    JA(1).rld = JA(nsho).rlg-JA(nback).rlg;
end

end