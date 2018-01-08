function[IMU] = EulerfromQuaternionIMUload(IMU)

for ii = 1:size(IMU,2)
    for jj = 1:length(IMU(ii).q.q0)
        q1.w = IMU(ii).q.q0(jj);
        q1.x = IMU(ii).q.q1(jj);
        q1.y = IMU(ii).q.q2(jj);
        q1.z = IMU(ii).q.q3(jj);
        
        test = q1.x*q1.y + q1.z*q1.w;
        
        sqw = q1.w*q1.w;
        sqx = q1.x*q1.x;
        sqy = q1.y*q1.y;
        sqz = q1.z*q1.z;
        
        unit = sqx + sqy + sqz + sqw;

        if (test > 0.499*unit) %% singularity at north pole
            IMU(ii).q.yw(jj) = 2 * atan2(q1.x,q1.w);
            IMU(ii).q.pt(jj) = pi/2;
            IMU(ii).q.rl(jj) = 0;
        elseif (test < -0.499*unit) %% singularity at south pole
            IMU(ii).q.yw(jj) = -2 * atan2(q1.x,q1.w);
            IMU(ii).q.pt(jj) = - pi/2;
            IMU(ii).q.rl(jj) = 0;
        else
            IMU(ii).q.yw(jj) = atan2(2*q1.y*q1.w-2*q1.x*q1.z , sqx - sqy - sqz + sqw);
            IMU(ii).q.pt(jj) = asin(2*test/unit);
            IMU(ii).q.rl(jj) = atan2(2*q1.x*q1.w-2*q1.y*q1.z , -sqx + sqy - sqz + sqw);
        end
    end
    IMU(ii).q.yw = rad2deg(IMU(ii).q.yw)';
    IMU(ii).q.pt = rad2deg(IMU(ii).q.pt)';
    IMU(ii).q.rl = rad2deg(IMU(ii).q.rl)';
    IMU(ii).q.ori = [IMU(ii).q.rl, IMU(ii).q.yw, IMU(ii).q.pt];
end
end
