function[JA] = getbody2IMUmat(IMU,tpose,calibtype)

% Get calibration indexes for different poses
for i = 1:length(tpose)
    ixn = ['ix',num2str(i)];
    [~,JA.ixp.(ixn)] = min(abs(IMU(1).stimem-tpose(i)));
end

switch calibtype
    case 'FE'
        % Calibration with IMU accelerations - sho FE
        for ii = 1:size(IMU,2)
            zsgA = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
            zsgB1 = mean(IMU(ii).acc(JA(1).ixp.ix3:JA(1).ixp.ix4,:));
            zsgB = zsgB1/norm(zsgB1);
            zsb = zsgA/norm(zsgA);
            xsb = -cross(zsb,zsgB)/norm(cross(zsb,zsgB));
            ysb = -cross(zsb,xsb)/norm(cross(zsb,xsb));
            
            JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
        end
        
    case 'AA'
        % Calibration with IMU accelerations - sho AA
        for ii = 1:size(IMU,2)
            
            zsgA = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
            zsgB1 = mean(IMU(ii).acc(JA(1).ixp.ix5:JA(1).ixp.ix6,:));
            zsgB = zsgB1/norm(zsgB1);
            zsb = zsgA/norm(zsgA);
            ysb = -cross(zsb,zsgB)/norm(cross(zsb,zsgB));
            xsb = cross(zsb,ysb)/norm(cross(zsb,ysb));
            
            JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
        end
        
    case 'FE+AA'
        % Calibration with IMU accelerations - sho FE+AA 
        for ii = 1:size(IMU,2)
            
            vsgA1 = mean(IMU(ii).acc(JA(1).ixp.ix1:JA(1).ixp.ix2,:));
            vsgB1 = mean(IMU(ii).acc(JA(1).ixp.ix5:JA(1).ixp.ix6,:));
            vsgC1 = mean(IMU(ii).acc(JA(1).ixp.ix3:JA(1).ixp.ix4,:));
            vsgA = vsgA1/norm(vsgA1);
            vsgB = vsgB1/norm(vsgB1);
            vsgC = vsgC1/norm(vsgC1);
            
            ysb = -cross(vsgA,vsgB)/norm(cross(vsgA,vsgB));
            xsb = cross(vsgA,vsgC)/norm(cross(vsgA,vsgC));
            zsb = cross(xsb,ysb)/norm(cross(xsb,ysb));
            
            JA(ii).Rsb = [xsb' ysb' zsb']; % Body to sensor matrix
        end
end
end